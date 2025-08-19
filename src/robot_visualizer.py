from __future__ import annotations

import numpy as np
from typing import Dict, Any, List

from yourdfpy import URDF
import pyroki as pk
import viser
from viser.extras import ViserUrdf

from utils import RobotConfig, IKSolver, load_urdf_with_fallback


class RobotVisualizer:

    def __init__(
        self,
        urdf_path: str,
        port: int,
        end_effectors: List[str] = None,
        custom_joints: List[tuple] = None,
        change_tolerance: float = 0.01,
        joint_limits: Dict[str, tuple] = None,
        label: str | None = None,
    ):
        self.server = viser.ViserServer(port=port, label=label)
        self.urdf = load_urdf_with_fallback(urdf_path)

        preprocessed_urdf = self._load_urdf_for_pyroki(self.urdf)
        self.robot = pk.Robot.from_urdf(preprocessed_urdf)
        self.robot_config = RobotConfig(change_tolerance=change_tolerance, viser_urdf=self._create_viser_urdf(), custom_joints=custom_joints, joint_limits=joint_limits)
        self.enable_server = False

        if end_effectors is None:
            end_effectors = []
        
        target_link_ids = self._create_target_links(end_effectors)
        self.ik_solver = IKSolver(self.robot, end_effectors, target_link_ids)

        self._setup_ui()
        if self.ik_solver.is_enabled():
            self._initialize_ik_targets()
        self.robot_config.print_robot_info()

    def _load_urdf_for_pyroki(self, urdf):
        actuated_joint_names = [
            name for name, joint in urdf.joint_map.items() if joint.type != "fixed"
        ]
        for name, joint in urdf.joint_map.items():
            if joint.mimic is not None:
                mimicked_joint_name = joint.mimic.joint
                if mimicked_joint_name not in actuated_joint_names:
                    print(
                        f"Warning: Removing mimic property from {name} (references {mimicked_joint_name})"
                    )
                    joint.mimic = None
        return urdf

    def _create_target_links(self, end_effectors: List[str]):
        """Create target link IDs for the given end effectors."""
        link_names = list(self.urdf.link_map.keys())
        target_link_ids = []
        
        for end_effector in end_effectors:
            link_idx = link_names.index(end_effector)
            target_link_ids.append(link_idx)
        
        return target_link_ids

    def _initialize_ik_targets(self):
        """Initialize IK targets using the IK solver."""
        current_config = self.robot_config.get_config(is_custom=False)
        self.ik_solver.initialize_targets(self.server, current_config, self._on_ik_target_moved)
        target_positions, target_wxyzs = self.ik_solver.get_target_data()
        if len(target_positions) > 0:
            solution = self.ik_solver.solve_ik(target_positions, target_wxyzs)
            if solution is not None:
                self._update_robot_with_solution(solution)

    def _on_ik_target_moved(self, _=None):
        """Handle IK target movement."""
        if not self.ik_solver.is_enabled():
            return
            
        target_positions, target_wxyzs = self.ik_solver.get_target_data()
        if len(target_positions) > 0:
            solution = self.ik_solver.solve_ik(target_positions, target_wxyzs)
            if solution is not None:
                self._update_robot_with_solution(solution)

    def _update_robot_with_solution(self, solution: np.ndarray):
        """Update robot configuration with IK solution."""
        joint_names = list(self.robot_config.get_sliders(is_custom=False).keys())
        joint_values = {joint_names[i]: solution[i] for i in range(len(solution))}
        result = self.robot_config.update_multiple_values(joint_values, is_custom=False)
        
        if not result["success"]:
            print("⚠️  IK solution applied with some issues.")

    def _create_viser_urdf(self) -> ViserUrdf:
        """Create ViserUrdf instance for visualization."""
        viser_urdf = ViserUrdf(
            self.server,
            urdf_or_path=self.urdf,
            load_meshes=True,
            load_collision_meshes=True,
            collision_mesh_color_override=(1.0, 0.0, 0.0, 0.5),
        )
        viser_urdf.show_collision = False
        return viser_urdf

    def _setup_ui(self):
        """Setup the user interface components."""
        with self.server.gui.add_folder("Joint position control"):
            for joint_name, (
                lower,
                upper,
            ) in self.robot_config.get_viser_urdf().get_actuated_joint_limits().items():
                lower = lower if lower is not None else -np.pi
                upper = upper if upper is not None else np.pi
                
                # Check if custom joint limits are provided for this joint
                if joint_name in self.robot_config.joint_limits:
                    custom_lower, custom_upper = self.robot_config.joint_limits[joint_name]
                    # Use the more restrictive limits (smaller range within URDF bounds)
                    if custom_lower >= lower and custom_upper <= upper:
                        lower = custom_lower
                        upper = custom_upper
                    else:
                        print(f"⚠️  Warning: Custom limits for joint '{joint_name}' [{custom_lower:.3f}, {custom_upper:.3f}] exceed URDF limits [{lower:.3f}, {upper:.3f}]. Using URDF limits.")
                initial_pos = (lower + upper) / 2.0

                slider = self.server.gui.add_slider(
                    label=joint_name,
                    min=lower,
                    max=upper,
                    step=1e-3,
                    initial_value=initial_pos,
                )
                slider.on_update(lambda _: self._update_robot_config())

                self.robot_config.add_slider(joint_name, slider, initial_pos, is_custom=False)

        # Create custom joint sliders if any are specified
        if self.robot_config.custom_joints:
            with self.server.gui.add_folder("Custom Joint Control"):
                for joint_name, lower_limit, upper_limit in self.robot_config.custom_joints:
                    initial_value = (lower_limit + upper_limit) / 2.0
                    
                    slider = self.server.gui.add_slider(
                        label=joint_name,
                        min=lower_limit,
                        max=upper_limit,
                        step=0.01,
                        initial_value=initial_value,
                    )
                    slider.on_update(lambda _: self._update_robot_config())
                    
                    self.robot_config.add_slider(joint_name, slider, initial_value, is_custom=True)
                    
                    print(f"🔧 Created custom joint slider: {joint_name} [{lower_limit:.3f}, {upper_limit:.3f}]")

        with self.server.gui.add_folder("Settings"):
            # IK Enable/Disable checkbox
            enable_ik_cb = self.server.gui.add_checkbox(
                "Enable IK", 
                self.ik_solver.is_enabled()
            )
            
            show_meshes_cb = self.server.gui.add_checkbox(
                "Show meshes",
                self.robot_config.get_viser_urdf().show_visual,
            )
            show_collision_meshes_cb = self.server.gui.add_checkbox(
                "Show collision meshes", self.robot_config.get_viser_urdf().show_collision
            )
            enable_server_cb = self.server.gui.add_checkbox(
                "Publish Joints", self.robot_config.get_publish_joints()
            )

            @enable_ik_cb.on_update
            def _(_):
                self._toggle_ik(enable_ik_cb.value)

            @show_meshes_cb.on_update
            def _(_):
                self.robot_config.get_viser_urdf().show_visual = show_meshes_cb.value

            @show_collision_meshes_cb.on_update
            def _(_):
                self.robot_config.get_viser_urdf().show_collision = show_collision_meshes_cb.value

            @enable_server_cb.on_update
            def _(_):
                self.robot_config.set_publish_joints(enable_server_cb.value)

        with self.server.gui.add_folder("Settings"):
            tolerance_input = self.server.gui.add_number(
                label="Change Tolerance",
                min=1e-8,
                max=1e-2,
                step=1e-8,
                initial_value=self.robot_config.get_change_tolerance(),
            )

            @tolerance_input.on_update
            def _(_):
                self.robot_config.set_change_tolerance(tolerance_input.value)
                print(f"🔄 Updated change tolerance to: {tolerance_input.value:.2e}")



        self.robot_config.update_viser_config(self.robot_config.initial_config)

    def _toggle_ik(self, enabled: bool):
        """Enable or disable IK functionality."""
        if enabled:
            print("✅ IK enabled")
        else:
            print("❌ IK disabled")
        
        self.ik_solver.enabled = enabled

    def _update_robot_config(self):
        """Update robot configuration when sliders change."""
        current_config = self.robot_config.get_config(is_custom=False)
        self.ik_solver.update_targets(current_config)
        self.robot_config.update_viser_config(current_config)




