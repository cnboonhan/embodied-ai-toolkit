from __future__ import annotations

from typing import Dict, List, Any, Tuple, Optional, Sequence
import numpy as np
import viser
from tabulate import tabulate
import os

# IK solving imports
import jax
import jax.numpy as jnp
import jax_dataclasses as jdc
import jaxlie
import jaxls
import numpy as onp
import pyroki as pk
from robot_descriptions.loaders.yourdfpy import load_robot_description
from yourdfpy import URDF


def load_urdf_with_fallback(urdf_path: str) -> URDF:
    """
    Load URDF from path if it exists, otherwise try to load from robot_descriptions.
    
    Args:
        urdf_path: Path to URDF file or robot description name
        
    Returns:
        URDF object
        
    Raises:
        FileNotFoundError: If URDF file doesn't exist and robot_descriptions fails
        ValueError: If robot_descriptions name is invalid
    """
    # Check if the path exists as a file
    if os.path.isfile(urdf_path):
        print(f"📁 Loading URDF from file: {urdf_path}")
        return URDF.load(urdf_path, load_collision_meshes=True, build_collision_scene_graph=True)
    
    # If not a file or directory, try robot_descriptions
    print(f"🤖 Attempting to load from robot_descriptions: {urdf_path}")
    try:
        return load_robot_description(urdf_path)
    except Exception as e:
        raise FileNotFoundError(
            f"URDF path '{urdf_path}' not found as file/directory and failed to load from robot_descriptions: {e}"
        )

def parse_custom_joints(
    custom_joints_str: str | None,
) -> List[Tuple[str, float, float]] | None:
    """
    Parse custom joints from a comma-separated string.

    Args:
        custom_joints_str: String in format "joint_name1:lower1:upper1,joint_name2:lower2:upper2"

    Returns:
        List of (joint_name, lower_limit, upper_limit) tuples, or None if no custom joints
        
    Raises:
        ValueError: If the input format is invalid or contains invalid numeric values
    """
    if not custom_joints_str:
        return None

    custom_joints_list = []
    for joint_spec in custom_joints_str.split(","):
        parts = joint_spec.strip().split(":")
        if len(parts) != 3:
            raise ValueError(
                f"Invalid custom joint format '{joint_spec}', expected 'name:lower:upper'"
            )
        
        joint_name = parts[0].strip()
        if not joint_name:
            raise ValueError(f"Empty joint name in '{joint_spec}'")
            
        try:
            lower_limit = float(parts[1].strip())
            upper_limit = float(parts[2].strip())
        except ValueError as e:
            raise ValueError(
                f"Invalid numeric limits for custom joint '{joint_name}': {e}"
            )
            
        if lower_limit >= upper_limit:
            raise ValueError(
                f"Invalid limits for custom joint '{joint_name}': lower limit ({lower_limit}) must be less than upper limit ({upper_limit})"
            )
            
        custom_joints_list.append((joint_name, lower_limit, upper_limit))

    return custom_joints_list if custom_joints_list else None


def parse_joint_limits(
    joint_limits_str: str | None,
) -> Dict[str, Tuple[float, float]] | None:
    """
    Parse joint limits from a comma-separated string.

    Args:
        joint_limits_str: String in format "joint_name1:lower1:upper1,joint_name2:lower2:upper2"

    Returns:
        Dictionary mapping joint names to (lower_limit, upper_limit) tuples, or None if no joint limits
        
    Raises:
        ValueError: If the input format is invalid or contains invalid numeric values
    """
    if not joint_limits_str:
        return None

    joint_limits_dict = {}
    for joint_spec in joint_limits_str.split(","):
        parts = joint_spec.strip().split(":")
        if len(parts) != 3:
            raise ValueError(
                f"Invalid joint limit format '{joint_spec}', expected 'name:lower:upper'"
            )
        
        joint_name = parts[0].strip()
        if not joint_name:
            raise ValueError(f"Empty joint name in '{joint_spec}'")
            
        try:
            lower_limit = float(parts[1].strip())
            upper_limit = float(parts[2].strip())
        except ValueError as e:
            raise ValueError(
                f"Invalid numeric limits for joint '{joint_name}': {e}"
            )
            
        if lower_limit >= upper_limit:
            raise ValueError(
                f"Invalid limits for joint '{joint_name}': lower limit ({lower_limit}) must be less than upper limit ({upper_limit})"
            )
            
        if joint_name in joint_limits_dict:
            raise ValueError(f"Duplicate joint name '{joint_name}' in joint limits")
            
        joint_limits_dict[joint_name] = (lower_limit, upper_limit)

    return joint_limits_dict if joint_limits_dict else None


def solve_ik_with_multiple_targets(
    robot: pk.Robot,
    target_link_names: Sequence[str],
    target_wxyzs: onp.ndarray,
    target_positions: onp.ndarray,
) -> onp.ndarray:
    """
    Solves the basic IK problem for a robot.

    Args:
        robot: PyRoKi Robot.
        target_link_names: Sequence[str]. List of link names to be controlled.
        target_wxyzs: onp.ndarray. Shape: (num_targets, 4). Target orientations.
        target_positions: onp.ndarray. Shape: (num_targets, 3). Target positions.

    Returns:
        cfg: onp.ndarray. Shape: (robot.joint.actuated_count,).
    """
    num_targets = len(target_link_names)
    assert target_positions.shape == (num_targets, 3)
    assert target_wxyzs.shape == (num_targets, 4)
    target_link_indices = [robot.links.names.index(name) for name in target_link_names]

    cfg = _solve_ik_jax(
        robot,
        jnp.array(target_wxyzs),
        jnp.array(target_positions),
        jnp.array(target_link_indices),
    )
    assert cfg.shape == (robot.joints.num_actuated_joints,)

    return onp.array(cfg)


@jdc.jit
def _solve_ik_jax(
    robot: pk.Robot,
    target_wxyz: jax.Array,
    target_position: jax.Array,
    target_joint_indices: jax.Array,
) -> jax.Array:
    JointVar = robot.joint_var_cls

    # Get the batch axes for the variable through the target pose.
    # Batch axes for the variables and cost terms (e.g., target pose) should be broadcastable!
    target_pose = jaxlie.SE3.from_rotation_and_translation(
        jaxlie.SO3(target_wxyz), target_position
    )
    batch_axes = target_pose.get_batch_axes()

    factors = [
        pk.costs.pose_cost_analytic_jac(
            jax.tree.map(lambda x: x[None], robot),
            JointVar(jnp.full(batch_axes, 0)),
            target_pose,
            target_joint_indices,
            pos_weight=50.0,
            ori_weight=10.0,
        ),
        pk.costs.rest_cost(
            JointVar(0),
            rest_pose=JointVar.default_factory(),
            weight=1.0,
        ),
        pk.costs.limit_cost(
            robot,
            JointVar(0),
            jnp.array([100.0] * robot.joints.num_joints),
        ),
    ]
    sol = (
        jaxls.LeastSquaresProblem(factors, [JointVar(0)])
        .analyze()
        .solve(
            verbose=False,
            linear_solver="dense_cholesky",
            trust_region=jaxls.TrustRegionConfig(lambda_initial=10.0),
        )
    )
    return sol[JointVar(0)]


class ExponentialSmoother:
    """Handles exponential smoothing with history for joint value smoothing."""

    def __init__(self, alpha: float = 0.1, history_length: int = 15):
        """
        Initialize the exponential smoother.

        Args:
            alpha: Smoothing factor (0.0-1.0). Lower values = smoother, higher values = more responsive
            history_length: Number of historical values to maintain
        """
        self.alpha = alpha
        self.history_length = history_length
        self.joint_history: Dict[str, List[float]] = {}

    def smooth_value(self, joint_name: str, new_value: float) -> float:
        """
        Apply exponential smoothing using all history values for better noise reduction.

        Args:
            joint_name: Name of the joint being smoothed
            new_value: New value to add to history and smooth

        Returns:
            Smoothed value using exponential weighting of all history
        """
        if joint_name not in self.joint_history:
            # Initialize with the new value repeated
            self.joint_history[joint_name] = [new_value] * self.history_length
            return new_value

        # Add new value to history
        self.joint_history[joint_name].append(new_value)

        # Keep only the last N values
        if len(self.joint_history[joint_name]) > self.history_length:
            self.joint_history[joint_name] = self.joint_history[joint_name][
                -self.history_length :
            ]

        # Use all values in history for smoothing
        # Apply exponential weights: more recent values get higher weight
        history_length = len(self.joint_history[joint_name])
        weights = [
            self.alpha * (1 - self.alpha) ** (history_length - 1 - i)
            for i in range(history_length)
        ]

        # Normalize weights to sum to 1
        total_weight = sum(weights)
        if total_weight > 0:
            normalized_weights = [w / total_weight for w in weights]
        else:
            # Fallback to equal weights if all weights are zero
            normalized_weights = [1.0 / history_length] * history_length

        # Calculate weighted average using all history values
        smoothed_value = sum(
            w * v for w, v in zip(normalized_weights, self.joint_history[joint_name])
        )

        return smoothed_value

    def get_history(self, joint_name: str) -> List[float]:
        """
        Get the current history for a joint.

        Args:
            joint_name: Name of the joint

        Returns:
            List of historical values for the joint
        """
        return self.joint_history.get(joint_name, [])

    def clear_history(self, joint_name: str = None):
        """
        Clear history for a specific joint or all joints.

        Args:
            joint_name: Name of the joint to clear, or None to clear all
        """
        if joint_name is None:
            self.joint_history.clear()
        else:
            self.joint_history.pop(joint_name, None)

    def update_parameters(self, alpha: float = None, history_length: int = None):
        """
        Update smoothing parameters.

        Args:
            alpha: New smoothing factor
            history_length: New history length
        """
        if alpha is not None:
            self.alpha = alpha
        if history_length is not None:
            self.history_length = history_length
            # Truncate existing histories if needed
            for joint_name in self.joint_history:
                if len(self.joint_history[joint_name]) > history_length:
                    self.joint_history[joint_name] = self.joint_history[joint_name][
                        -history_length:
                    ]

    def get_stats(self) -> Dict[str, any]:
        """
        Get statistics about the smoother.

        Returns:
            Dictionary with smoothing statistics
        """
        total_joints = len(self.joint_history)
        total_values = sum(len(history) for history in self.joint_history.values())

        return {
            "alpha": self.alpha,
            "history_length": self.history_length,
            "total_joints": total_joints,
            "total_values": total_values,
            "average_values_per_joint": (
                total_values / total_joints if total_joints > 0 else 0
            ),
        }


class RobotConfig:
    """Manages robot joint configuration, including sliders, limits, and update logic."""

    def __init__(
        self,
        change_tolerance: float = 1e-6,
        viser_urdf=None,
        custom_joints: List[tuple] = None,
        smoother=None,
        joint_limits: Dict[str, tuple] = None,
    ):
        """
        Initialize the robot configuration manager.

        Args:
            change_tolerance: Minimum change threshold for joint updates
            viser_urdf: ViserUrdf instance for robot visualization
            custom_joints: List of (joint_name, lower_limit, upper_limit) tuples for custom joints
            smoother: ExponentialSmoother instance for applying smoothing to joint updates
            joint_limits: Dictionary mapping joint names to (lower_limit, upper_limit) tuples for custom joint limits
        """
        self.change_tolerance = change_tolerance
        self.viser_urdf = viser_urdf
        self.custom_joints = custom_joints or []
        self.smoother = smoother
        self.joint_limits = joint_limits or {}

        self.joint_sliders: Dict[str, viser.GuiInputHandle[float]] = {}
        self.custom_sliders: Dict[str, viser.GuiInputHandle[float]] = {}
        self.initial_config: List[float] = []
        self.custom_initial_config: List[float] = []

    def add_slider(
        self,
        name: str,
        slider: viser.GuiInputHandle[float],
        initial_value: float,
        is_custom: bool = False,
    ):
        """
        Add a slider to the configuration.

        Args:
            name: Name of the joint or custom control
            slider: Viser slider handle (contains min/max limits)
            initial_value: Initial value
            is_custom: Whether this is a custom control (True) or URDF joint (False)
        """
        if is_custom:
            self.custom_sliders[name] = slider
            self.custom_initial_config.append(initial_value)
        else:
            self.joint_sliders[name] = slider
            self.initial_config.append(initial_value)

    def get_config(self, is_custom: bool = False) -> List[float]:
        """Get current configuration as a list of values."""
        sliders = self.custom_sliders if is_custom else self.joint_sliders
        return [slider.value for slider in sliders.values()]

    def get_value(self, name: str, is_custom: bool = False) -> float:
        """Get current value of a specific joint or custom control."""
        sliders = self.custom_sliders if is_custom else self.joint_sliders
        control_type = "custom control" if is_custom else "joint"

        if name not in sliders:
            raise ValueError(f"{control_type.title()} '{name}' not found")
        return sliders[name].value

    def validate_value(
        self, name: str, value: float, is_custom: bool = False
    ) -> Tuple[bool, str, float]:
        """
        Validate a value against limits and clamp if necessary.

        Args:
            name: Name of the joint or custom control
            value: Value to validate
            is_custom: Whether this is a custom control (True) or URDF joint (False)

        Returns:
            Tuple of (is_valid, error_message, clamped_value)
        """
        # Get the appropriate slider dictionary
        sliders = self.custom_sliders if is_custom else self.joint_sliders
        control_type = "custom control" if is_custom else "joint"

        if name not in sliders:
            return False, f"{control_type.title()} '{name}' not found", value

        slider = sliders[name]
        lower_limit, upper_limit = slider.min, slider.max

        if value < lower_limit or value > upper_limit:
            clamped_value = max(lower_limit, min(upper_limit, value))
            return (
                False,
                f"Value {value} is outside {control_type} limits [{lower_limit}, {upper_limit}] for {control_type} {name}.",
                clamped_value,
            )

        return True, "", value

    def has_value_changed(
        self, name: str, new_value: float, is_custom: bool = False
    ) -> bool:
        """
        Check if a value has changed significantly.

        Args:
            name: Name of the joint or custom control
            new_value: New value to check
            is_custom: Whether this is a custom control (True) or URDF joint (False)

        Returns:
            True if the value has changed beyond tolerance
        """
        current_value = self.get_value(name, is_custom=is_custom)
        return abs(new_value - current_value) > self.change_tolerance

    def update_value(
        self, name: str, value: float, is_custom: bool = False
    ) -> Dict[str, Any]:
        """
        Update a joint or custom value with validation, smoothing, and change detection.

        Args:
            name: Name of the joint or custom control
            value: New value
            is_custom: Whether this is a custom control (True) or URDF joint (False)

        Returns:
            Dictionary with update results
        """
        current_value = self.get_value(name, is_custom=is_custom)

        # Apply smoothing if smoother is available
        if self.smoother is not None:
            smoothed_value = self.smoother.smooth_value(name, value)
        else:
            smoothed_value = value

        # Validate and clamp value
        is_valid, error_msg, final_value = self.validate_value(
            name, smoothed_value, is_custom=is_custom
        )

        if not is_valid and error_msg:
            sliders = self.custom_sliders if is_custom else self.joint_sliders
            slider = sliders[name]
            control_type = "custom joint" if is_custom else "joint"
            print(f"❌ Error updating {control_type} '{name}': {error_msg}")
            return {
                "success": False,
                "error": error_msg,
                "value": current_value,
                "requested_value": value,
                "final_value": final_value,
                "limits": [slider.min, slider.max],
            }

        # Check if value has changed
        if not self.has_value_changed(name, final_value, is_custom=is_custom):
            sliders = self.custom_sliders if is_custom else self.joint_sliders
            slider = sliders[name]
            return {
                "success": True,
                "skipped": True,
                "reason": "No change detected",
                "value": current_value,
                "requested_value": value,
                "final_value": final_value,
                "limits": [slider.min, slider.max],
            }

        # Update the slider
        sliders = self.custom_sliders if is_custom else self.joint_sliders
        sliders[name].value = final_value

        slider = sliders[name]
        return {
            "success": True,
            "updated": True,
            "previous_value": current_value,
            "final_value": final_value,
            "limits": [slider.min, slider.max],
        }

    def update_multiple_values(
        self, values: Dict[str, float], is_custom: bool = False
    ) -> Dict[str, Any]:
        """
        Update multiple joint or custom values at once.

        Args:
            values: Dictionary mapping joint/custom names to values
            is_custom: Whether these are custom controls (True) or URDF joints (False)

        Returns:
            Dictionary with update results for all values
        """
        results = {}
        updated_values = []

        for name, value in values.items():
            result = self.update_value(name, value, is_custom=is_custom)
            results[name] = result

            if result.get("updated", False):
                updated_values.append(name)

        # Update viser configuration if any URDF joints were updated
        if not is_custom and updated_values:
            current_config = self.get_config(is_custom=False)
            self.update_viser_config(current_config)

        return {
            "success": all(result.get("success", False) for result in results.values()),
            "results": results,
            (
                "updated_joints" if not is_custom else "updated_custom_joints"
            ): updated_values,
        }

    def get_info(self, is_custom: bool = False) -> Dict[str, Any]:
        """Get information about all joints or custom controls."""
        sliders = self.custom_sliders if is_custom else self.joint_sliders
        info = {}
        for name, slider in sliders.items():
            info[name] = {
                "value": slider.value,
                "limits": [slider.min, slider.max],
            }
        return info



    def get_sliders(
        self, is_custom: bool = False
    ) -> Dict[str, viser.GuiInputHandle[float]]:
        """Get slider handles for joints or custom controls."""
        sliders = self.custom_sliders if is_custom else self.joint_sliders
        return sliders.copy()

    def set_change_tolerance(self, tolerance: float):
        """Set the change tolerance for detecting significant changes."""
        self.change_tolerance = tolerance

    def get_change_tolerance(self) -> float:
        """Get the current change tolerance."""
        return self.change_tolerance

    def update_viser_config(self, config: List[float]):
        """Update the ViserUrdf configuration with URDF joint values only."""
        if self.viser_urdf is not None:
            self.viser_urdf.update_cfg(config)

    def get_viser_urdf(self):
        """Get the ViserUrdf instance."""
        return self.viser_urdf

    def print_robot_info(self):
        """Print detailed information about the robot using viser_urdf data."""
        if self.viser_urdf is None:
            print("❌ No ViserUrdf instance available")
            return

        # Get robot information from viser_urdf
        urdf = self.viser_urdf._urdf
        actuated_joints = [
            (name, joint)
            for name, joint in urdf.joint_map.items()
            if joint.type != "fixed"
        ]

        print(f"📊 Total joints: {len(urdf.joint_map)}")
        print(f"🔗 Total links: {len(urdf.link_map)}")
        print(f"\n🎯 Actuated joints ({len(actuated_joints)}):")
        if actuated_joints:
            csv_data = [["ID", "Joint Name", "Type", "Lower Limit", "Upper Limit"]]
            for i, (joint_name, joint) in enumerate(actuated_joints, 1):
                if joint.limit:
                    lower_limit = f"{joint.limit.lower:.3f}"
                    upper_limit = f"{joint.limit.upper:.3f}"
                else:
                    lower_limit = "No limit"
                    upper_limit = "No limit"
                csv_data.append(
                    [str(i), joint_name, joint.type, lower_limit, upper_limit]
                )
            self._print_csv_as_table(csv_data)

        print(f"\n🔗 All links ({len(urdf.link_map)}):")
        if urdf.link_map:
            # Create CSV data for links
            link_csv_data = [["ID", "Link Name"]]
            for i, (link_name, link) in enumerate(urdf.link_map.items(), 1):
                link_csv_data.append([str(i), link_name])

            self._print_csv_as_table(link_csv_data)

    def _print_csv_as_table(self, csv_data):
        """Print CSV data as a formatted table using tabulate."""
        if not csv_data:
            return

        # Extract headers and data
        headers = csv_data[0]
        data = csv_data[1:]

        # Print table using tabulate with grid format
        print(tabulate(data, headers=headers, tablefmt="grid"))


class IKSolver:
    """Handles inverse kinematics functionality and data management."""

    def __init__(self, robot, target_link_names: List[str], target_link_ids: List[int]):
        """
        Initialize the IK solver.

        Args:
            robot: Pyroki robot instance
            target_link_names: List of IK target link names
            target_link_ids: List of corresponding link IDs
        """
        self.robot = robot
        self.target_link_names = target_link_names
        self.target_link_ids = target_link_ids
        self.ik_targets: List[Any] = [None] * len(target_link_names)
        self.enabled = len(target_link_names) > 0

    def initialize_targets(self, server, current_config: List[float], on_target_moved_callback=None):
        """
        Initialize IK targets in the viser scene.

        Args:
            server: Viser server instance
            current_config: Current robot configuration
            on_target_moved_callback: Callback function to call when targets are moved
        """
        if not self.enabled:
            print("⚠️  IK disabled - no IK target link names provided")
            return

        current_config_array = np.array(current_config)

        # Initialize IK targets for all targets
        for i, (target_name, link_id) in enumerate(
            zip(self.target_link_names, self.target_link_ids)
        ):
            transform = self.robot.forward_kinematics(current_config_array, link_id)
            if transform is not None:
                position = np.array(transform[link_id, -3:])
                quaternion = np.array(transform[link_id, :4])

                self.ik_targets[i] = server.scene.add_transform_controls(
                    f"/ik_target_{target_name}",
                    scale=0.2,
                    position=position,
                    wxyz=tuple(quaternion),
                )
                
                # Add movement callback if provided
                if on_target_moved_callback is not None:
                    self.ik_targets[i].on_update(on_target_moved_callback)

    def update_targets(self, current_config: List[float]):
        """
        Update IK target positions based on current robot configuration.

        Args:
            current_config: Current robot configuration
        """
        current_config_array = np.array(current_config)

        for i, (target_name, link_id) in enumerate(
            zip(self.target_link_names, self.target_link_ids)
        ):
            if not self.enabled:
                transform = self.robot.forward_kinematics(current_config_array, link_id)
                if transform is not None and self.ik_targets[i] is not None:
                    position = np.array(transform[link_id, -3:])
                    self.ik_targets[i].position = position

                    quaternion = np.array(transform[link_id, :4])
                    self.ik_targets[i].wxyz = tuple(quaternion)

    def get_target_data(self) -> Tuple[np.ndarray, np.ndarray]:
        """
        Get current target positions and orientations.

        Returns:
            Tuple of (target_positions, target_wxyzs)
        """
        if not self.enabled or not all(
            target is not None for target in self.ik_targets
        ):
            return np.array([]), np.array([])

        target_positions = np.array([target.position for target in self.ik_targets])
        target_wxyzs = np.array([target.wxyz for target in self.ik_targets])

        return target_positions, target_wxyzs

    def solve_ik(
        self, target_positions: np.ndarray, target_wxyzs: np.ndarray
    ) -> np.ndarray | None:
        """
        Solve inverse kinematics and return the solution.

        Args:
            target_positions: Target positions for IK targets
            target_wxyzs: Target orientations for IK targets

        Returns:
            Solution array if successful, None if failed
        """
        try:
            solution = solve_ik_with_multiple_targets(
                robot=self.robot,
                target_link_names=self.target_link_names,
                target_positions=target_positions,
                target_wxyzs=target_wxyzs,
            )
            return solution

        except Exception as e:
            print(f"❌ IK solver failed: {e}")
            return None

    def is_enabled(self) -> bool:
        """Check if IK is enabled."""
        return self.enabled

    def get_target_count(self) -> int:
        """Get the number of IK targets."""
        return len(self.target_link_names)

    def get_target_names(self) -> List[str]:
        """Get the list of target link names."""
        return self.target_link_names.copy()

def apply_joint_reversal_from_limits(joint_value: float, limits: list) -> float:
    """
    Apply joint value reversal using limits array format.
    
    Args:
        joint_value: The original joint value to reverse
        limits: List containing [lower_limit, upper_limit]
        
    Returns:
        The reversed joint value mirrored around the limits midpoint
        
    Raises:
        ValueError: If limits list doesn't have at least 2 elements
    """
    if len(limits) < 2:
        raise ValueError("Limits array must have at least 2 elements [lower, upper]")
    
    lower_limit = limits[0]
    upper_limit = limits[1]
    
    # Mirror the value around the midpoint of joint limits
    midpoint = (lower_limit + upper_limit) / 2
    distance_from_midpoint = joint_value - midpoint
    reversed_value = midpoint - distance_from_midpoint
    return reversed_value


