from dataclasses import dataclass

from lerobot.teleoperators.config import TeleoperatorConfig


@TeleoperatorConfig.register_subclass("urdf_leader")
@dataclass
class UrdfLeaderConfig(TeleoperatorConfig):
    grpc_endpoint: str
    motor_whitelist: list[str]
