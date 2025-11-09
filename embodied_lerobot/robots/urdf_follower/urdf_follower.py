import logging
import time
import json
import subprocess
from functools import cached_property
from typing import Any

from lerobot.cameras.utils import make_cameras_from_configs
from .config_urdf_follower import UrdfFollowerConfig
from lerobot.robots.robot import Robot

logger = logging.getLogger(__name__)


class UrdfFollower(Robot):
    config_class = UrdfFollowerConfig
    name = "urdf_follower"

    def __init__(self, config: UrdfFollowerConfig):
        super().__init__(config)
        self.config = config
        self.motors = config.motor_whitelist
        self.cameras = make_cameras_from_configs(config.cameras)

    def _get_robot_info(self):
        command = ["grpcurl", "-plaintext", "-format", "json", self.config.grpc_endpoint, "rosbot_api.RobotApiService/GetRobotInfo" ]
        result = subprocess.run(command, capture_output=True, text=True, check=True)
        all_data = json.loads(result.stdout.strip())["joint_positions"]
        # TODO(BH): Verify if all keys in self.motors are present in data
        return {key: all_data[key] for key in all_data if key in self.motors}

    def _send_joint_update(self, goal_pos):
        payload = {
            "joint_updates": goal_pos
        }
        command = [
            "grpcurl",
            "-plaintext",
            "-d",
            json.dumps(payload),
            self.config.grpc_endpoint,
            "rosbot_api.RobotApiService/UpdateJoints",
        ]
        subprocess.run(command, capture_output=True, text=True, check=True)

    @property
    def _motors_ft(self) -> dict[str, type]:
        return {f"{motor}.pos": float for motor in self.motors}

    @property
    def _cameras_ft(self) -> dict[str, tuple]:
        return {
            cam: (self.config.cameras[cam].height, self.config.cameras[cam].width, 3) for cam in self.cameras
        }

    @cached_property
    def observation_features(self) -> dict[str, type | tuple]:
        return {**self._motors_ft, **self._cameras_ft}

    @cached_property
    def action_features(self) -> dict[str, type]:
        return self._motors_ft

    @property
    def is_connected(self) -> bool:
        # TODO(BH): Verify connected
        return all(cam.is_connected for cam in self.cameras.values())

    def connect(self, calibrate: bool = True) -> None:
        # TODO(BH): Verify Connection
        for cam in self.cameras.values():
            cam.connect()

    @property
    def is_calibrated(self) -> bool:
        return True

    def calibrate(self) -> None:
        return

    def configure(self) -> None:
        return

    def setup_motors(self) -> None:
        return

    def get_observation(self) -> dict[str, Any]:
        obs_dict = {}

        robot_info = self._get_robot_info()
        obs_dict = {f"{motor}.pos": val for motor, val in robot_info.items()}

        for cam_key, cam in self.cameras.items():
            start = time.perf_counter()
            obs_dict[cam_key] = cam.async_read()
            dt_ms = (time.perf_counter() - start) * 1e3
            logger.debug(f"{self} read {cam_key}: {dt_ms:.1f}ms")
        
        return obs_dict

    def send_action(self, action: dict[str, Any]) -> dict[str, Any]:
        goal_pos = {key.removesuffix(".pos"): val for key, val in action.items() if key.endswith(".pos")}
        self._send_joint_update(goal_pos)
        return {f"{motor}.pos": val for motor, val in goal_pos.items()}

    def disconnect(self):
        for cam in self.cameras.values():
            cam.disconnect()