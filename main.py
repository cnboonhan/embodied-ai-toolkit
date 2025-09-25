# python main.py --config_path example_config.json

import tyro
import time
import viser
from pathlib import Path
from src.utils import load_urdf, load_config, setup_ui, ApiServer
from viser.extras import ViserUrdf

def main(config_path: Path):

    config = load_config(config_path)

    viser_server = viser.ViserServer(port=config.urdf_viewer_port, verbose=False, label=config.label)
    urdf_robot = ViserUrdf(viser_server, load_urdf(config.urdf_path), load_meshes=True, load_collision_meshes=False)
    slider_handles, slider_names, custom_slider_handles, custom_slider_names = setup_ui(viser_server, urdf_robot, config)

    api_server = ApiServer(
        slider_handles,
        slider_names,
        custom_slider_handles,
        custom_slider_names,
        urdf_robot,
        project_name=config.project_name,
        api_port=config.api_port
    )

    
    while True:
        time.sleep(1.0 / 30.0)

if __name__ == "__main__":
    tyro.cli(main)

