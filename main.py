# python main.py --config_path example_config.json

import tyro
import time
import viser
from pathlib import Path
from src.utils import load_urdf, load_config, setup_ui, ApiServer
from viser.extras import ViserUrdf
import rerun as rr

def main(config_path: Path):

    config = load_config(config_path)

    rr.init(config.project_name, spawn=False)
    rr.serve_grpc(grpc_port=config.data_grpc_port)
    rr.serve_web_viewer(open_browser=True, web_port=config.data_viewer_port, connect_to=f"rerun+http://localhost:{config.data_grpc_port}/proxy")

    viser_server = viser.ViserServer(port=config.urdf_viewer_port)
    urdf_robot = ViserUrdf(viser_server, load_urdf(config.urdf_path), load_meshes=True, load_collision_meshes=False)
    slider_handles, slider_names, custom_slider_handles, custom_slider_names = setup_ui(viser_server, urdf_robot, config)

    api_server = ApiServer(
        slider_handles,
        slider_names,
        custom_slider_handles,
        custom_slider_names,
        urdf_robot,
        port=config.api_port,
    )
    while True:
        time.sleep(10.0)

if __name__ == "__main__":
    tyro.cli(main)

