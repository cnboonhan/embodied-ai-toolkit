# curl http://127.0.0.1:5000/get_joints
# x86; aima em stop-app motion_player

import rclpy
import requests
import json
import time
from collections import deque
from rclpy.node import Node
from sensor_msgs.msg import JointState


class JointStateReceiver(Node):
    def __init__(self):
        super().__init__("joint_state_receiver")
        self.server_url = "http://127.0.0.1:5000/get_joints"
        self.latest_joint_state = None
        self.joint_names = []
        self.joint_positions = []
        self.joint_velocities = []
        self.joint_efforts = []
        
        # Smoothing configuration - can be overridden by ROS parameters
        self.declare_parameter('exponential_alpha', 0.1)
        self.declare_parameter('history_length', 10)
        self.declare_parameter('change_tolerance', 0.008)
        self.declare_parameter('max_change_per_step', 0.04)
        #self.declare_parameter('change_tolerance', 0.22)
        #self.declare_parameter('max_change_per_step', 0.22)
        
        self.exponential_alpha = self.get_parameter('exponential_alpha').value
        self.history_length = self.get_parameter('history_length').value
        self.change_tolerance = self.get_parameter('change_tolerance').value
        self.max_change_per_step = self.get_parameter('max_change_per_step').value
        
        # Smoothing state
        self.joint_history = {}  # Store smoothed values for each joint
        self.smoothing_stats = {
            "total_updates": 0,
            "total_skipped": 0,
            "last_update_time": None
        }
        self.arm_subscription = self.create_subscription(
            JointState,
            "/motion/control/arm_joint_state",
            self.arm_joint_state_callback,
            10,
        )
        self.get_logger().info("Subscribed to /motion/control/arm_joint_state")
        
        # Create publisher for joint commands
        self.joint_command_publisher = self.create_publisher(
            JointState,
            "/motion/control/arm_joint_command",
            10,
        )
        self.get_logger().info("Publisher created for /motion/control/arm_joint_command")
        
        self.timer = self.create_timer(0.01, self.timer_callback)
        self.get_logger().info("Timer created - will print joint state every 0.01 seconds")
        
        # Log smoothing configuration
        self.get_logger().info(f"Smoothing configuration:")
        self.get_logger().info(f"  Exponential alpha: {self.exponential_alpha}")
        self.get_logger().info(f"  History length: {self.history_length}")
        self.get_logger().info(f"  Change tolerance: {self.change_tolerance}")
        self.get_logger().info(f"  Max change per step: {self.max_change_per_step}")

    def arm_joint_state_callback(self, msg):
        """Callback function for arm joint state messages"""
        # Save the latest joint state
        self.latest_joint_state = msg
        self.joint_names = list(msg.name)
        self.joint_positions = list(msg.position)
        self.joint_velocities = list(msg.velocity)
        self.joint_efforts = list(msg.effort)

    def apply_exponential_smoothing(self, joint_name: str, new_value: float) -> float:
        """Apply exponential smoothing with extended history for better noise reduction."""
        if joint_name not in self.joint_history:
            # Initialize with the new value repeated
            self.joint_history[joint_name] = [new_value] * self.history_length
            return new_value
        
        # Get the last smoothed value
        last_smoothed = self.joint_history[joint_name][-1]
        
        # Apply exponential smoothing: smoothed = α * new + (1-α) * previous
        smoothed_value = self.exponential_alpha * new_value + (1 - self.exponential_alpha) * last_smoothed
        
        # Store the smoothed value
        self.joint_history[joint_name].append(smoothed_value)
        
        # Keep only the last N values to maintain history length
        if len(self.joint_history[joint_name]) > self.history_length:
            self.joint_history[joint_name] = self.joint_history[joint_name][-self.history_length:]
        
        return smoothed_value

    def apply_velocity_limiting(self, current_value: float, target_value: float) -> float:
        """Apply velocity limiting to prevent sudden large changes."""
        difference = target_value - current_value
        
        # Check if change is within tolerance
        if abs(difference) <= self.change_tolerance:
            return current_value  # No change needed
        
        # Limit the maximum change per step
        if abs(difference) > self.max_change_per_step:
            if difference > 0:
                return current_value + self.max_change_per_step
            else:
                return current_value - self.max_change_per_step
        
        return target_value

    def get_smoothing_stats(self) -> dict:
        """Get smoothing statistics."""
        stats = self.smoothing_stats.copy()
        
        # Calculate efficiency metrics
        total_requests = stats["total_updates"] + stats["total_skipped"]
        if total_requests > 0:
            stats["update_efficiency"] = stats["total_updates"] / total_requests
            stats["skip_efficiency"] = stats["total_skipped"] / total_requests
        else:
            stats["update_efficiency"] = 0.0
            stats["skip_efficiency"] = 0.0
        
        return stats

    def reconfigure_smoothing(self, exponential_alpha: float = None, history_length: int = None, 
                            change_tolerance: float = None, max_change_per_step: float = None):
        """Reconfigure smoothing parameters dynamically."""
        if exponential_alpha is not None:
            self.exponential_alpha = exponential_alpha
            self.get_logger().info(f"Updated exponential_alpha to {exponential_alpha}")
        
        if history_length is not None:
            self.history_length = history_length
            self.get_logger().info(f"Updated history_length to {history_length}")
        
        if change_tolerance is not None:
            self.change_tolerance = change_tolerance
            self.get_logger().info(f"Updated change_tolerance to {change_tolerance}")
        
        if max_change_per_step is not None:
            self.max_change_per_step = max_change_per_step
            self.get_logger().info(f"Updated max_change_per_step to {max_change_per_step}")
        
        # Clear joint history when parameters change
        self.joint_history.clear()
        self.get_logger().info("Cleared joint history due to parameter change")

    def timer_callback(self):
        """Timer callback to print the latest joint state message"""
        try:
            response = requests.get(self.server_url, timeout=5)
            response.raise_for_status()
            json_data = response.json()
            self.get_logger().info(
                f"HTTP Response JSON: {json.dumps(json_data, indent=2)}"
            )
            
            # Extract and process all joint values from the response
            # Create a copy of the latest joint state once before the loop
            updated_joint_state = None
            if self.latest_joint_state is not None:
                updated_joint_state = JointState()
                updated_joint_state.header = self.latest_joint_state.header
                updated_joint_state.name = list(self.latest_joint_state.name)
                updated_joint_state.position = list(self.latest_joint_state.position)
                updated_joint_state.velocity = list(self.latest_joint_state.velocity)
                updated_joint_state.effort = list(self.latest_joint_state.effort)
            
            for joint_key, joint_data in json_data.items():
                # Skip joints idx01 to idx12
                skip_joints = ['idx01', 'idx02', 'idx03', 'idx04', 'idx05', 'idx06', 
                              'idx07', 'idx08', 'idx09', 'idx10', 'idx11', 'idx12']
                if any(joint_key.startswith(skip_joint) for skip_joint in skip_joints):
                    self.get_logger().info(f"Skipping {joint_key} (idx01-idx12)")
                    continue
                
                # Only process joints that start with 'idx13' to 'idx26'
                idx13_to_26 = ['idx13', 'idx14', 'idx15', 'idx16', 'idx17', 'idx18', 'idx19', 'idx20', 
                               'idx21', 'idx22', 'idx23', 'idx24', 'idx25', 'idx26']
                if not any(joint_key.startswith(idx_joint) for idx_joint in idx13_to_26):
                    self.get_logger().info(f"Skipping {joint_key} (not idx13-idx26)")
                    continue
                
                self.get_logger().info(f"Processing joint: {joint_key}")
                
                # Handle case where joint_data might be a dictionary
                if isinstance(joint_data, dict):
                    # Try to extract position value from the dictionary
                    if 'value' in joint_data:
                        joint_value = joint_data['value']
                    else:
                        self.get_logger().error(f"Could not find current value in joint data: {joint_data}")
                        continue
                else:
                    self.get_logger().error(f"Joint data is not a dictionary: {joint_data}")
                    continue
                
                self.get_logger().info(f"Using joint value for {joint_key}: {joint_value}")
                
                # Update the joint state if it exists
                if updated_joint_state is not None:
                    # Find the index of the joint in the joint names
                    joint_index = None
                    for i, name in enumerate(updated_joint_state.name):
                        if name == joint_key:
                            joint_index = i
                            break
                    
                    # Update the position value if the joint was found
                    if joint_index is not None:
                        current_value = updated_joint_state.position[joint_index]
                        
                        # Apply exponential smoothing to reduce noise
                        smoothed_value = self.apply_exponential_smoothing(joint_key, joint_value)
                        
                        # Apply velocity limiting to prevent sudden large changes
                        final_value = self.apply_velocity_limiting(current_value, smoothed_value)
                        
                        # Check if the value actually changed
                        if abs(final_value - current_value) > self.change_tolerance:
                            updated_joint_state.position[joint_index] = final_value
                            self.smoothing_stats["total_updates"] += 1
                            
                            # Log detailed smoothing information
                            self.get_logger().info(f"🔄 {joint_key}: {current_value:.4f} → {smoothed_value:.4f} → {final_value:.4f}")
                        else:
                            self.smoothing_stats["total_skipped"] += 1
                            self.get_logger().debug(f"⏭️ {joint_key}: No significant change ({current_value:.4f} ≈ {final_value:.4f})")
                    else:
                        self.get_logger().warning(f"{joint_key} not found in joint names")
                else:
                    self.get_logger().info("No joint state available to update")
            
            # Update statistics
            self.smoothing_stats["last_update_time"] = time.time()
            
            # Publish the updated joint state to the command topic if we have updates
            if updated_joint_state is not None:
                self.joint_command_publisher.publish(updated_joint_state)
                self.get_logger().info(f"Published updated joint state to /motion/control/arm_joint_command")
                
                # Log statistics periodically (every 100 updates)
                total_requests = self.smoothing_stats["total_updates"] + self.smoothing_stats["total_skipped"]
                if total_requests > 0 and total_requests % 100 == 0:
                    stats = self.get_smoothing_stats()
                    self.get_logger().info(f"📊 Smoothing Stats: {stats['total_updates']} updates, {stats['total_skipped']} skipped, "
                                          f"Efficiency: {stats['update_efficiency']:.1%}")
                
        except requests.exceptions.RequestException as e:
            self.get_logger().error(f"Failed to make HTTP request: {e}")
        except json.JSONDecodeError as e:
            self.get_logger().error(f"Failed to parse JSON response: {e}")
        except Exception as e:
            self.get_logger().error(f"Unexpected error in timer callback: {e}")


def main(args=None):
    rclpy.init(args=args)
    node = JointStateReceiver()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()