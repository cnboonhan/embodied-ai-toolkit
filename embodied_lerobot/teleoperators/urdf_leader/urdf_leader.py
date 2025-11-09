import logging
from functools import cached_property
import subprocess
import json

from lerobot.teleoperators.teleoperator import Teleoperator
from .config_urdf_leader import UrdfLeaderConfig

logger = logging.getLogger(__name__)


class UrdfLeader(Teleoperator):
    config_class = UrdfLeaderConfig
    name = "urdf_leader"

    def __init__(self, config: UrdfLeaderConfig):
        super().__init__(config)
        self.config = config
        self.motors = config.motor_whitelist

    def _get_robot_info(self):
        command = ["grpcurl", "-plaintext", "-format", "json", self.config.grpc_endpoint, "rosbot_api.RobotApiService/GetRobotInfo" ]
        result = subprocess.run(command, capture_output=True, text=True, check=True)
        all_data = json.loads(result.stdout.strip())["joint_positions"]
        # TODO(BH): Verify if all keys in self.motors are present in data
        return {key: all_data[key] for key in all_data if key in self.motors}

    @property
    def _motors_ft(self) -> dict[str, type]:
        return {f"{motor}.pos": float for motor in self.motors}

    @cached_property
    def action_features(self) -> dict[str, type]:
        return self._motors_ft

    @cached_property
    def feedback_features(self) -> dict[str, type]:
        return {}

    @property
    def is_connected(self) -> bool:
        # TODO(BH): Verify connected
        return True

    def connect(self, calibrate: bool = True) -> None:
        # TODO(BH): Verify Connection
        return True

    @property
    def is_calibrated(self) -> bool:
        return True

    def calibrate(self) -> None:
        return True

    def configure(self) -> None:
        return True

    def setup_motors(self) -> None:
        return True

    def get_action(self) -> dict[str, float]:
        action_dict = {}

        robot_info = self._get_robot_info()
        action_dict = {f"{motor}.pos": val for motor, val in robot_info.items()}

        return action_dict

    def send_feedback(self, feedback: dict[str, float]) -> None:
        return

    def disconnect(self) -> None:
        return