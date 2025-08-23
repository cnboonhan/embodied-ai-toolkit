from robot_descriptions.loaders.yourdfpy import load_robot_description
from pathlib import Path
import yourdfpy
import numpy as np
import viser
from viser.extras import ViserUrdf
from dataclasses import dataclass
from typing import List, Optional, Tuple
import json
import asyncio
import threading
import time
from datetime import datetime
import grpc
from concurrent import futures

# Import generated gRPC stubs
from src import api_pb2
from src import api_pb2_grpc


@dataclass
class CustomJoint:
    name: str
    type: str
    limits: Tuple[float, float]


@dataclass
class Config:
    project_name: str
    urdf_path: str
    api_port: int
    urdf_viewer_port: int
    data_viewer_port: int
    data_grpc_port: int
    custom_joints: Optional[List[CustomJoint]] = None


def load_config(config_path: Path) -> Config:
    import json

    with open(config_path, "r") as f:
        data = json.load(f)

    # Convert custom_joints if present
    custom_joints = None
    if "custom_joints" in data:
        custom_joints = [
            CustomJoint(
                name=joint["name"], type=joint["type"], limits=tuple(joint["limits"])
            )
            for joint in data["custom_joints"]
        ]

    return Config(
        project_name=data["project_name"],
        urdf_path=data["urdf_path"],
        api_port=data["api_port"],
        urdf_viewer_port=data["urdf_viewer_port"],
        data_viewer_port=data["data_viewer_port"],
        data_grpc_port=data["data_grpc_port"],
        custom_joints=custom_joints,
    )


def load_urdf(urdf_path: str):
    try:
        if Path(urdf_path).exists():
            return yourdfpy.URDF.load(urdf_path)
        else:
            return load_robot_description(urdf_path)
    except Exception as e:
        raise Exception(
            f"Failed to load URDF from local or robot_descriptions '{urdf_path}'"
        )


def setup_ui(server: viser.ViserServer, robot: ViserUrdf, config: Config):
    slider_names: list[str] = []
    slider_handles: list[viser.GuiInputHandle[float]] = []
    custom_slider_handles: list[viser.GuiInputHandle[float]] = []
    custom_slider_names: list[str] = []

    with server.gui.add_folder("Joint position controls"):
        for joint_name, (
            lower,
            upper,
        ) in robot.get_actuated_joint_limits().items():
            lower = lower if lower is not None else -np.pi
            upper = upper if upper is not None else np.pi
            initial_pos = (lower + upper) / 2.0

            slider = server.gui.add_slider(
                label=joint_name,
                min=lower,
                max=upper,
                step=1e-3,
                initial_value=initial_pos,
            )
            slider.on_update(
                lambda _, s=slider: robot.update_cfg(
                    np.array([slider.value for slider in slider_handles])
                )
            )
            slider_handles.append(slider)
            slider_names.append(joint_name)

    if config.custom_joints:
        with server.gui.add_folder("Custom position controls"):
            for custom_joint in config.custom_joints:
                lower, upper = custom_joint.limits
                initial_pos = (lower + upper) / 2.0

                slider = server.gui.add_slider(
                    label=custom_joint.name,
                    min=lower,
                    max=upper,
                    step=1e-3,
                    initial_value=initial_pos,
                )
                custom_slider_handles.append(slider)
                custom_slider_names.append(custom_joint.name)

    robot.update_cfg(np.array([slider.value for slider in slider_handles]))
    return slider_handles, slider_names, custom_slider_handles, custom_slider_names


class RobotApiServicer(api_pb2_grpc.RobotApiServiceServicer):
    """gRPC servicer implementation for robot API"""

    def __init__(self, api_server):
        self.api_server = api_server
        self._streaming_clients = set()
        self._streaming_active = False
        self._streaming_thread = None

    def UpdateJoints(self, request, context):
        """Update joint positions"""
        try:
            # Convert request maps to regular dicts
            joint_updates = dict(request.joint_updates)
            custom_joint_updates = dict(request.custom_joint_updates)

            # Validate and update joints
            updated_joints = []
            invalid_joints = []

            # Update regular joints
            for joint_name, angle in joint_updates.items():
                if joint_name in self.api_server.slider_names:
                    joint_index = self.api_server.slider_names.index(joint_name)
                    slider = self.api_server.slider_handles[joint_index]

                    if slider.min <= angle <= slider.max:
                        slider.value = angle
                        updated_joints.append(joint_name)
                    else:
                        invalid_joints.append(joint_name)
                else:
                    invalid_joints.append(joint_name)

            for joint_name, angle in custom_joint_updates.items():
                if joint_name in self.api_server.custom_slider_names:
                    joint_index = self.api_server.custom_slider_names.index(joint_name)
                    slider = self.api_server.custom_slider_handles[joint_index]

                    if slider.min <= angle <= slider.max:
                        slider.value = angle
                        updated_joints.append(joint_name)
                    else:
                        invalid_joints.append(joint_name)
                else:
                    invalid_joints.append(joint_name)

            if self.api_server.slider_handles:
                config = [slider.value for slider in self.api_server.slider_handles]
                self.api_server.robot.update_cfg(np.array(config))

            return api_pb2.UpdateJointsResponse(
                updated_joints=updated_joints, invalid_joints=invalid_joints
            )

        except Exception as e:
            context.set_code(grpc.StatusCode.INTERNAL)
            context.set_details(f"Error updating joints: {str(e)}")
            return api_pb2.UpdateJointsResponse(
                updated_joints=[],
                invalid_joints=list(joint_updates.keys())
                + list(custom_joint_updates.keys()),
            )

    def GetRobotInfo(self, request, context):
        """Get comprehensive robot information"""
        try:
            joint_positions = {}
            for i, joint_name in enumerate(self.api_server.slider_names):
                joint_positions[joint_name] = self.api_server.slider_handles[i].value

            custom_joint_positions = {}
            for i, joint_name in enumerate(self.api_server.custom_slider_names):
                custom_joint_positions[joint_name] = (
                    self.api_server.custom_slider_handles[i].value
                )

            joint_limits = {}
            for i, slider in enumerate(self.api_server.slider_handles):
                joint_limits[self.api_server.slider_names[i]] = api_pb2.JointLimits(
                    lower=slider.min,
                    upper=slider.max,
                    joint_type="revolute",  # Default assumption
                )

            custom_joint_limits = {}
            for i, slider in enumerate(self.api_server.custom_slider_handles):
                custom_joint_limits[self.api_server.custom_slider_names[i]] = (
                    api_pb2.JointLimits(
                        lower=slider.min,
                        upper=slider.max,
                        joint_type="revolute",  # Default assumption
                    )
                )

            return api_pb2.GetRobotInfoResponse(
                project_name=self.api_server.project_name,
                joint_names=self.api_server.slider_names,
                custom_joint_names=self.api_server.custom_slider_names,
                total_joints=len(self.api_server.slider_names),
                total_custom_joints=len(self.api_server.custom_slider_names),
                joint_positions=joint_positions,
                custom_joint_positions=custom_joint_positions,
                joint_limits=joint_limits,
                custom_joint_limits=custom_joint_limits,
            )

        except Exception as e:
            context.set_code(grpc.StatusCode.INTERNAL)
            context.set_details(f"Error getting robot info: {str(e)}")
            return api_pb2.GetRobotInfoResponse()

    def StreamRobotState(self, request, context):
        """Stream robot state updates"""
        try:
            # Add client to streaming set
            self._streaming_clients.add(context)

            # Start streaming thread if not already running
            if not self._streaming_active:
                self._streaming_active = True
                self._streaming_thread = threading.Thread(
                    target=self._stream_robot_state, daemon=True
                )
                self._streaming_thread.start()

            # Stream data to this client
            update_frequency = (
                request.update_frequency if request.update_frequency > 0 else 5.0
            )
            interval = 1.0 / update_frequency

            while context.is_active():
                try:
                    # Create robot state message
                    robot_state = self._create_robot_state_message(request)
                    yield robot_state

                    # Wait for next update
                    time.sleep(interval)

                except Exception as e:
                    print(f"Error streaming to client: {e}")
                    break

            # Remove client from streaming set
            self._streaming_clients.discard(context)

        except Exception as e:
            context.set_code(grpc.StatusCode.INTERNAL)
            context.set_details(f"Error in streaming: {str(e)}")

    def _create_robot_state_message(self, request):
        """Create a RobotState message"""
        # Get current joint positions
        joint_positions = {}
        for i, joint_name in enumerate(self.api_server.slider_names):
            joint_positions[joint_name] = self.api_server.slider_handles[i].value

        custom_joint_positions = {}
        for i, joint_name in enumerate(self.api_server.custom_slider_names):
            custom_joint_positions[joint_name] = self.api_server.custom_slider_handles[
                i
            ].value

        # Always create robot configuration
        robot_config = api_pb2.RobotConfiguration(
            joint_names=self.api_server.slider_names,
            custom_joint_names=self.api_server.custom_slider_names,
            total_joints=len(self.api_server.slider_names),
            total_custom_joints=len(self.api_server.custom_slider_names),
        )

        timestamp = datetime.now()
        timestamp_proto = api_pb2.Timestamp()
        timestamp_proto.FromDatetime(timestamp)

        return api_pb2.RobotState(
            joint_positions=joint_positions,
            custom_joint_positions=custom_joint_positions,
            robot_config=robot_config,
            timestamp=timestamp_proto,
            project_name=self.api_server.project_name,
        )

    def _stream_robot_state(self):
        """Background thread for streaming robot state"""
        while self._streaming_active:
            # This method can be used for broadcasting to all clients
            # Currently each client has its own streaming loop
            time.sleep(0.1)


class ApiServer:
    def __init__(
        self,
        slider_handles: List[viser.GuiInputHandle[float]],
        slider_names: List[str],
        custom_slider_handles: List[viser.GuiInputHandle[float]],
        custom_slider_names: List[str],
        robot: ViserUrdf,
        project_name: str,
        api_port: int = 5000,
        data_uri: str = "rerun+http://localhost:5001/proxy",
    ):
        self.slider_handles = slider_handles
        self.slider_names = slider_names
        self.custom_slider_handles = custom_slider_handles
        self.custom_slider_names = custom_slider_names
        self.robot = robot
        self.project_name = project_name
        self.api_port = api_port
        self.data_uri = data_uri

        # Initialize gRPC server
        self.grpc_server = None
        self.grpc_servicer = None
        self._start_grpc_server()

    def _start_grpc_server(self):
        """Start the gRPC server"""
        try:
            # Create gRPC server
            self.grpc_server = grpc.server(futures.ThreadPoolExecutor(max_workers=10))

            # Create and add servicer
            self.grpc_servicer = RobotApiServicer(self)
            api_pb2_grpc.add_RobotApiServiceServicer_to_server(
                self.grpc_servicer, self.grpc_server
            )

            # Start server
            server_address = f"[::]:{self.api_port}"
            self.grpc_server.add_insecure_port(server_address)
            self.grpc_server.start()

            print(f"✓ gRPC server started on port {self.api_port}")

        except Exception as e:
            print(f"✗ Failed to start gRPC server: {e}")
            self.grpc_server = None

    def stop_grpc_server(self):
        """Stop the gRPC server"""
        if self.grpc_server:
            self.grpc_server.stop(0)
            print("✓ gRPC server stopped")
