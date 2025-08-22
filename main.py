# python main.py --config_path example_config.json

from dataclasses import dataclass
import tyro
import time
import viser
from pathlib import Path
from src.utils import load_urdf, load_config, setup_ui
from viser.extras import ViserUrdf

def main(config_path: Path):
    
    config = load_config(config_path)
    server = viser.ViserServer(port=config.visualization_port)
    robot = ViserUrdf(server, load_urdf(config.urdf_path), load_meshes=True, load_collision_meshes=False)
    joint_handles = setup_ui(server, robot, config)

    while True:
        time.sleep(10.0)

if __name__ == "__main__":
    tyro.cli(main)

