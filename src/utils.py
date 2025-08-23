from robot_descriptions.loaders.yourdfpy import load_robot_description
from pathlib import Path
import yourdfpy
import numpy as np
import viser
from viser.extras import ViserUrdf
from dataclasses import dataclass
from typing import List, Optional, Tuple
import json


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


class ApiServer:
    def __init__(
        self,
        slider_handles: List[viser.GuiInputHandle[float]],
        slider_names: List[str],
        custom_slider_handles: List[viser.GuiInputHandle[float]],
        custom_slider_names: List[str],
        robot: ViserUrdf,
        port: int = 5000,
    ):
        self.slider_handles = slider_handles
        self.slider_names = slider_names
        self.custom_slider_handles = custom_slider_handles
        self.custom_slider_names = custom_slider_names
        self.robot = robot
        self.port = port
        

    # async def broadcast_joint_update(
    #     self, joints_to_publish=[], custom_joints_to_publish=[]
    # ):
    #     joints = {}

    #     for joint_name in joints_to_publish:
    #         if joint_name in self.slider_names:
    #             joint_index = self.slider_names.index(joint_name)
    #             joints[joint_name] = self.slider_handles[joint_index].value

    #     for joint_name in custom_joints_to_publish:
    #         if joint_name in self.custom_slider_names:
    #             joint_index = self.custom_slider_names.index(joint_name)
    #             joints[joint_name] = self.custom_slider_handles[joint_index].value

    #     if joints and self.pub:
    #         try:
    #             message = json.dumps(joints)
    #             await self.pub.put(self.joint_status_topic, message.encode())
    #         except Exception as e:
    #             print(f"Error broadcasting joint updates: {e}")

    # async def handle_message(self, message):
    #     """Handle incoming ZeroMQ messages"""
    #     try:
    #         data = json.loads(message)
    #         command = data.get("command")
            
    #         if command == "get_joints":
    #             joints = {}
    #             for i, joint_name in enumerate(self.slider_names):
    #                 joints[joint_name] = self.slider_handles[i].value
    #             for i, joint_name in enumerate(self.custom_slider_names):
    #                 joints[joint_name] = self.custom_slider_handles[i].value
    #             return {"success": True, "data": joints}

    #         elif command == "get_limits":
    #             limits = {}
    #             for i, slider in enumerate(self.slider_handles):
    #                 limits[self.slider_names[i]] = {
    #                     "lower": slider.min,
    #                     "upper": slider.max,
    #                 }
    #             for i, slider in enumerate(self.custom_slider_handles):
    #                 limits[self.custom_slider_names[i]] = {
    #                     "lower": slider.min,
    #                     "upper": slider.max,
    #                 }
    #             return {"success": True, "data": limits}

    #         elif command == "update_joints":
    #             updates = data.get("joints", {})
                
    #             invalid_joints = []
    #             for joint_name, angle in updates.items():
    #                 if (
    #                     joint_name not in self.slider_names
    #                     and joint_name not in self.custom_slider_names
    #                 ):
    #                     invalid_joints.append(joint_name)

    #                 if joint_name in self.slider_names:
    #                     joint_index = self.slider_names.index(joint_name)
    #                     if (
    #                         angle < self.slider_handles[joint_index].min
    #                         or angle > self.slider_handles[joint_index].max
    #                     ):
    #                         invalid_joints.append(joint_name)

    #                 elif joint_name in self.custom_slider_names:
    #                     joint_index = self.custom_slider_names.index(joint_name)
    #                     if (
    #                         angle < self.custom_slider_handles[joint_index].min
    #                         or angle > self.custom_slider_handles[joint_index].max
    #                     ):
    #                         invalid_joints.append(joint_name)

    #             if invalid_joints:
    #                 return {"success": False, "error": f"Invalid joints: {invalid_joints}"}

    #             updated_joints = []
    #             for joint_name, angle in updates.items():
    #                 if joint_name in self.slider_names:
    #                     joint_index = self.slider_names.index(joint_name)
    #                     self.slider_handles[joint_index].value = angle
    #                     updated_joints.append(joint_name)

    #                 elif joint_name in self.custom_slider_names:
    #                     joint_index = self.custom_slider_names.index(joint_name)
    #                     self.custom_slider_handles[joint_index].value = angle
    #                     updated_joints.append(joint_name)

    #             if self.slider_handles:
    #                 config = [slider.value for slider in self.slider_handles]
    #                 self.robot.update_cfg(np.array(config))

    #             await self.broadcast_joint_update(updated_joints)
    #             return {"success": True, "updated_joints": updated_joints}
    #         else:
    #             return {"success": False, "error": f"Unknown command: {command}"}

    #     except json.JSONDecodeError:
    #         return {"success": False, "error": "Invalid JSON format"}
    #     except Exception as e:
    #         return {"success": False, "error": str(e)}
