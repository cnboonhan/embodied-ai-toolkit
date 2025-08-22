# python main.py --config_path example_config.json
# echo "get_joints" | websocat ws://localhost:5000/ws
# echo "get_limits" | websocat ws://localhost:5000/ws
# echo '/update_joint {"torso_joint": 0.5, "left_hip_pitch_joint": 1.0}' | websocat ws://localhost:5000/ws

from dataclasses import dataclass
import tyro
import time
import viser
from pathlib import Path
from src.utils import load_urdf, load_config, setup_ui, start_websocket_server
from viser.extras import ViserUrdf

def main(config_path: Path):
    
    config = load_config(config_path)
    server = viser.ViserServer(port=config.visualization_port)
    robot = ViserUrdf(server, load_urdf(config.urdf_path), load_meshes=True, load_collision_meshes=False)

    slider_handles, slider_names, custom_slider_handles, custom_slider_names = setup_ui(server, robot, config)
    websocket_server = start_websocket_server(slider_handles, slider_names, custom_slider_handles, custom_slider_names, robot, port=config.api_port)

    while True:
        time.sleep(10.0)

if __name__ == "__main__":
    tyro.cli(main)

