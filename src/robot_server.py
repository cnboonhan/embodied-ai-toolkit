from __future__ import annotations

import time
import threading

import tyro
from tabulate import tabulate

from robot_visualizer import RobotVisualizer
from websocket_api_server import WebSocketAPIServer
from utils import parse_custom_joints, parse_joint_limits


def main(
    path: str,
    vis_port: int,
    api_port: int,
    ik_targets: str | None = None,
    custom_joints: str | None = None,
    joint_limits: str | None = None,
    change_tolerance: float = 0.045,
    label: str | None = None,
) -> None:
    custom_joints_list = parse_custom_joints(custom_joints)
    joint_limits_dict = parse_joint_limits(joint_limits)
    ik_targets_list = None
    if ik_targets:
        ik_targets_list = [name.strip() for name in ik_targets.split(",")]
    
    robot_viz = RobotVisualizer(
        path, vis_port, ik_targets_list, custom_joints_list, change_tolerance, joint_limits_dict, label
    )
    api_server = WebSocketAPIServer(robot_viz.robot_config, api_port)
    
    robot_viz.broadcast_callback = api_server.broadcast_update_sync
    websocket_thread = threading.Thread(target=api_server.run, daemon=True)
    websocket_thread.start()

    print()
    print("🤖 Robot Visualization and Control Server")
    server_info = [
        ["📊 Viser visualization server", f"http://localhost:{vis_port}"],
        ["🔌 WebSocket API server", f"ws://localhost:{api_port}"],
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
    print("📋 Available WebSocket Endpoints")
    websocket_endpoints_data = [
        ["ws://localhost:{api_port}/ws/update_joints", "WebSocket endpoint for joint updates and queries"],
    ]
    
    print(tabulate(websocket_endpoints_data, headers=["Endpoint", "Description"], tablefmt="grid"))

    print()
    print("💡 WebSocket Usage:")
    print("  • Connect to ws://localhost:{api_port}/ws/update_joints")
    print("  • Send empty message or null to get current joint state")
    print("  • Send joint updates in format: {{\"joint_name\": {{\"value\": joint_value}}}}")
    print("  • Receive real-time updates when joints change")

    try:
        while True:
            time.sleep(10.0)
    except KeyboardInterrupt:
        print("\n🛑 Shutting down robot server...")
        print("👋 Goodbye!")


if __name__ == "__main__":
    tyro.cli(main)
