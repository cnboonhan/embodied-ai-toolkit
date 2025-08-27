# python main.py --config_path example_config.json

import tyro
import time
import viser
from pathlib import Path
from src.utils import load_urdf, load_config, setup_ui, ApiServer
from viser.extras import ViserUrdf
import rerun as rr

def main(config_path: Path, enable_rerun: bool = True):

    config = load_config(config_path)

    rr.init(config.project_name, spawn=False, recording_id=config.epsisode_name)
    if enable_rerun:
        rr.serve_grpc(grpc_port=config.data_grpc_port)
        rr.serve_web_viewer(open_browser=False, web_port=config.data_viewer_port, connect_to=f"rerun+http://{config.data_host}:{config.data_grpc_port}/proxy")
        print(f"Visit for Data View: http://{config.data_host}:{config.data_viewer_port}/?url=rerun%2Bhttp%3A%2F%2F{config.data_host}%3A{config.data_grpc_port}%2Fproxy")
    else:
        rr.connect_grpc(url=f"rerun+http://{config.data_host}:{config.data_grpc_port}/proxy")

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
        api_port=config.api_port,
        data_uri=f"rerun+http://{config.data_host}:{config.data_grpc_port}/proxy"
    )

    
    while True:
        time.sleep(1.0 / 30.0)

if __name__ == "__main__":
    tyro.cli(main)

