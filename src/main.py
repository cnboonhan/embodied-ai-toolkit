from __future__ import annotations

import time
import threading

import tyro
from tabulate import tabulate

from robot_visualizer import RobotVisualizer
from flask_api_server import FlaskAPIServer
from utils import parse_custom_joints, parse_joint_limits


def main(
    path: str,
    vis_port: int,
    api_port: int,
    ik_targets: str | None = None,
    custom_joints: str | None = None,
    joint_limits: str | None = None,
    exponential_alpha: float = 0.05,
    history_length: int = 15,
    change_tolerance: float = 0.01,
) -> None:
    custom_joints_list = parse_custom_joints(custom_joints)
    joint_limits_dict = parse_joint_limits(joint_limits)
    ik_targets_list = None
    if ik_targets:
        ik_targets_list = [name.strip() for name in ik_targets.split(",")]
    
    robot_viz = RobotVisualizer(
        path, vis_port, ik_targets_list, custom_joints_list, exponential_alpha, history_length, change_tolerance, joint_limits_dict
    )
    api_server = FlaskAPIServer(robot_viz.robot_config, api_port)
    flask_thread = threading.Thread(target=api_server.run, daemon=True)
    flask_thread.start()

    print()
    print("🤖 Robot Visualization and Control Server")
    server_info = [
        ["📊 Viser visualization server", f"http://localhost:{vis_port}"],
        ["🔌 Flask API server", f"http://localhost:{api_port}"],
        ["🔄 Exponential smoothing (α)", f"{exponential_alpha:.2f}"],
        ["📈 History length", f"{history_length} samples"],
        ["🎯 Change tolerance", f"{change_tolerance:.2e}"],
    ]

    if robot_viz.ik_solver.is_enabled():
        server_info.extend([
            ["🎯 Inverse Kinematics", f"Auto-solving for {robot_viz.ik_solver.get_target_count()} IK target(s)"],
            ["🎯 IK targets", ", ".join(robot_viz.ik_solver.get_target_names())],
        ])

    print(tabulate(server_info, headers=["Setting", "Value"], tablefmt="grid"))

    if custom_joints_list:
        print()
        print("🔧 Custom Joints")
        custom_joints_data = []
        for joint_name, lower, upper in custom_joints_list:
            custom_joints_data.append([joint_name, f"[{lower:.3f}, {upper:.3f}]"])
        print(tabulate(custom_joints_data, headers=["Joint Name", "Limits"], tablefmt="grid"))

    if joint_limits_dict:
        print()
        print("⚙️  Joint Limits Override")
        joint_limits_data = []
        for joint_name, (lower, upper) in joint_limits_dict.items():
            joint_limits_data.append([joint_name, f"[{lower:.3f}, {upper:.3f}]"])
        print(tabulate(joint_limits_data, headers=["Joint Name", "Custom Limits"], tablefmt="grid"))

    print()
    print("📋 Available API Endpoints")
    api_endpoints_data = [
        ["POST /update_joints", "Update one or more joints at once"],
        ["GET /get_joints", "Get all joint information (URDF + custom joints)"],
    ]
    
    print(tabulate(api_endpoints_data, headers=["Endpoint", "Description"], tablefmt="grid"))

    try:
        while True:
            time.sleep(10.0)
    except KeyboardInterrupt:
        print("\n🛑 Shutting down robot server...")
        print("👋 Goodbye!")


if __name__ == "__main__":
    tyro.cli(main)
