from robot_descriptions.loaders.yourdfpy import load_robot_description
from pathlib import Path
import yourdfpy
import numpy as np
import viser
from viser.extras import ViserUrdf
from dataclasses import dataclass
from typing import List, Optional, Tuple

@dataclass
class CustomJoint:
    name: str
    type: str
    limits: Tuple[float, float]

@dataclass
class Config:
    urdf_path: str
    api_port: int
    visualization_port: int
    custom_joints: Optional[List[CustomJoint]] = None

def load_config(config_path: Path) -> Config:
    import json
    
    with open(config_path, 'r') as f:
        data = json.load(f)
    
    # Convert custom_joints if present
    custom_joints = None
    if 'custom_joints' in data:
        custom_joints = [
            CustomJoint(
                name=joint['name'],
                type=joint['type'],
                limits=tuple(joint['limits'])
            )
            for joint in data['custom_joints']
        ]
    
    return Config(
        urdf_path=data['urdf_path'],
        api_port=data['api_port'],
        visualization_port=data['visualization_port'],
        custom_joints=custom_joints
    )

def load_urdf(urdf_path: str):
    try:
        if Path(urdf_path).exists():
            return yourdfpy.URDF.load(urdf_path)
        else: 
            return load_robot_description(urdf_path)
    except Exception as e:
        raise Exception(f"Failed to load URDF from local or robot_descriptions '{urdf_path}'")


def setup_ui(server: viser.ViserServer, robot: ViserUrdf, config: Config):
    slider_handles: list[viser.GuiInputHandle[float]] = []

    with server.gui.add_folder("Joint position controls"):
        for joint_name, (lower,upper,) in robot.get_actuated_joint_limits().items():
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
            slider.on_update(lambda _, s=slider: robot.update_cfg(np.array([slider.value for slider in slider_handles])))
            slider_handles.append(slider)

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
                # slider.on_update(lambda _, s=slider: robot.update_cfg(np.array(np.array([slider.value for slider in slider_handles]))))
                # slider_handles.append(slider)

    robot.update_cfg(np.array([slider.value for slider in slider_handles]))
    return slider_handles