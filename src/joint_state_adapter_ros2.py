import rclpy
import requests
import json
import time
import argparse
from typing import Optional, Dict, Any
from collections import deque
from rclpy.node import Node
from sensor_msgs.msg import JointState
from joint_state_adapter import JointStateAdapter, JointStateData, JointData, AdapterConfig


class ROS2JointStateAdapter(JointStateAdapter):
    def __init__(self, config: AdapterConfig):
        super().__init__(config)
        
        self.subscription_topic = config.messaging_config.get("subscription_topic", "/joint_states")
        self.publisher_topic = config.messaging_config.get("publisher_topic", "/joint_command")
        self.joint_name_mapping = self._parse_joint_name_mapping(config.joint_name_mapping)
        
        # Create inverse mapping: ROS joint name -> API joint name
        self.inverse_joint_name_mapping = {}
        for api_name, mapping_info in self.joint_name_mapping.items():
            ros_name = mapping_info['ros_name']
            self.inverse_joint_name_mapping[ros_name] = api_name
        
        self.node = None
        self.subscription = None
        self.publisher = None
        self.timer = None
        
        self.latest_robot_joint_state = None
        self.latest_server_joint_state = None
        
    def _parse_joint_name_mapping(self, joint_name_mapping_str: str) -> Dict[str, Dict[str, Any]]:
        """Parse joint name mapping string into dictionary."""
        joint_name_mapping = {}
        if joint_name_mapping_str.strip():
            for mapping in joint_name_mapping_str.split(","):
                if ":" in mapping:
                    parts = mapping.split(":")
                    api_name = parts[0].strip()
                    ros_name = parts[1].strip()
                    reverse = parts[2].strip().lower() == 'true' if len(parts) > 2 else False
                    joint_name_mapping[api_name] = {
                        'ros_name': ros_name,
                        'reverse': reverse
                    }
        return joint_name_mapping
    
    def _ros_joint_state_to_joint_state_data(self, ros_msg: JointState) -> JointStateData:
        """Convert ROS2 JointState message to JointStateData with proper JointData objects."""
        if not self.latest_server_joint_state:
            raise ValueError("Cannot convert ROS joint state: no server joint state data available")
        
        joints = {}
        timestamp = time.time()  # Use current time since ROS JointState doesn't have timestamp info we need
        
        for i, name in enumerate(ros_msg.name):
            if i < len(ros_msg.position):
                current_value = ros_msg.position[i]
                
                api_joint_name = self.inverse_joint_name_mapping.get(name, name)
                
                # Check if joint exists in server data using the original API joint name. If not mapped, skip.
                if api_joint_name not in self.latest_server_joint_state.joints:
                    continue
                
                # Use limits from server joint state using the original API joint name
                limits = self.latest_server_joint_state.joints[api_joint_name].limits
                
                joints[name] = JointData(
                    current_value=current_value,
                    limits=limits
                )
        
        return JointStateData(joints=joints, timestamp=timestamp)
    
    def _joint_state_data_to_ros_joint_state(self, joint_state_data: JointStateData, 
                                           base_msg: Optional[JointState] = None) -> JointState:
        """Convert JointStateData to ROS2 JointState message."""
        ros_msg = JointState()
        ros_msg.header.stamp = self.node.get_clock().now().to_msg() if self.node else None
        
        if base_msg:
            # Use base message structure but update positions
            ros_msg.header = base_msg.header
            ros_msg.name = list(base_msg.name)
            ros_msg.position = list(base_msg.position)
            ros_msg.velocity = list(base_msg.velocity)
            ros_msg.effort = list(base_msg.effort)
        else:
            # Create new message from joint state data
            ros_msg.name = list(joint_state_data.joints.keys())
            ros_msg.position = []
            ros_msg.velocity = []
            ros_msg.effort = []
            
            for joint_name in ros_msg.name:
                joint_data = joint_state_data.joints[joint_name]
                ros_msg.position.append(joint_data.current_value)
                ros_msg.velocity.append(0.0)  # Default velocity
                ros_msg.effort.append(0.0)    # Default effort
        
        return ros_msg
    
    def _robot_joint_state_callback(self, msg: JointState):
        """Callback function for robot joint state messages."""
        self.latest_robot_joint_state = msg
    
    def _timer_callback(self):
        """Timer callback to sync between server and robot."""
        try:
            # Get joint state from server
            server_joint_state = self.get_joint_state_from_server()
            if server_joint_state:
                # Save the latest server joint state for use in conversions
                self.latest_server_joint_state = server_joint_state
                
                # Add server joint state to history for exponential smoothing
                self.add_to_server_joint_state_history(server_joint_state)
                
                if self.latest_robot_joint_state:
                    self.sync_joint_state_to_robot(server_joint_state)
        except Exception as e:
            if self.node:
                self.node.get_logger().error(f"Error in timer callback: {e}")
    
    def get_joint_state_from_server(self) -> Optional[JointStateData]:
        """Get current joint state data from the HTTP API server."""
        try:
            response = requests.get(self.config.server_url, timeout=5)
            response.raise_for_status()
            json_data = response.json()
            
            if self.node:
                self.node.get_logger().debug(
                    f"HTTP Response JSON: {json.dumps(json_data, indent=2)}"
                )
            
            # Convert server response to JointStateData format with proper JointData objects
            joints = {}
            timestamp = time.time()
            
            for joint_name, joint_data in json_data.items():
                if isinstance(joint_data, dict) and "value" in joint_data and "limits" in joint_data:
                    try:
                        joints[joint_name] = JointData(
                            current_value=joint_data["value"],
                            limits=joint_data["limits"]
                        )
                    except ValueError as e:
                        if self.node:
                            self.node.get_logger().warning(f"Invalid joint data for {joint_name}: {e}")
                        continue
                else:
                    if self.node:
                        self.node.get_logger().warning(
                            f"Joint {joint_name} missing required fields (value, limits): {joint_data}"
                        )
            
            if not joints:
                if self.node:
                    self.node.get_logger().warning("No valid joints found in server response")
                return None
            
            return JointStateData(joints=joints, timestamp=timestamp)
            
        except requests.exceptions.RequestException as e:
            if self.node:
                self.node.get_logger().error(f"Failed to make HTTP request: {e}")
            return None
        except json.JSONDecodeError as e:
            if self.node:
                self.node.get_logger().error(f"Failed to parse JSON response: {e}")
            return None
        except Exception as e:
            if self.node:
                self.node.get_logger().error(f"Unexpected error getting server state: {e}")
            return None
    
    def get_joint_state_from_robot(self) -> Optional[JointStateData]:
        """Get current joint state data from the robot."""
        if self.latest_robot_joint_state:
            try:
                return self._ros_joint_state_to_joint_state_data(self.latest_robot_joint_state)
            except ValueError as e:
                if self.node:
                    self.node.get_logger().warning(f"Could not get robot joint state: {e}")
                return None
            except Exception as e:
                if self.node:
                    self.node.get_logger().error(f"Error getting robot joint state: {e}")
                return None
        return None
    
    def sync_joint_state_to_server(self, joint_state: JointStateData) -> None:
        """Sync/send joint state data to the server."""
        if self.node:
            self.node.get_logger().debug("sync_joint_state_to_server called (not implemented)")
    
    def sync_joint_state_to_robot(self, joint_state: JointStateData) -> None:
        """Send motion commands to control the robot to match the joint state."""
        if not self.latest_robot_joint_state or not self.publisher:
            return
        
        # Apply joint reversal first using the base class method
        processed_joint_state = self.apply_joint_reversal_with_mapping(
            self.joint_name_mapping, joint_state
        )
        
        # Apply exponential smoothing using the base class method
        smoothed_joint_state = self.apply_exponential_smoothing(processed_joint_state)
        
        # Create updated joint state based on current robot state
        updated_joint_state = JointState()
        updated_joint_state.header = self.latest_robot_joint_state.header
        updated_joint_state.header.stamp = self.node.get_clock().now().to_msg()
        updated_joint_state.name = list(self.latest_robot_joint_state.name)
        updated_joint_state.position = list(self.latest_robot_joint_state.position)
        updated_joint_state.velocity = list(self.latest_robot_joint_state.velocity)
        updated_joint_state.effort = list(self.latest_robot_joint_state.effort)
        
        # Process each joint from server data
        for api_joint_name, mapping_info in self.joint_name_mapping.items():
            if api_joint_name in smoothed_joint_state.joints:
                joint_data = smoothed_joint_state.joints[api_joint_name]
                ros_joint_name = mapping_info['ros_name']
                joint_value = joint_data.value
                
                # Find the joint index in the ROS message
                joint_index = None
                for i, name in enumerate(updated_joint_state.name):
                    if name == ros_joint_name:
                        joint_index = i
                        break
                
                if joint_index is not None:
                    current_value = updated_joint_state.position[joint_index]
                    
                    # Apply change tolerance check
                    if abs(joint_value - current_value) > self.config.change_tolerance:
                        updated_joint_state.position[joint_index] = joint_value
                        
                        # Log the update
                        reverse_indicator = " (reversed)" if mapping_info.get('reverse', False) else ""
                        original_value = joint_state.joints[api_joint_name].current_value if api_joint_name in joint_state.joints else joint_value
                        smoothing_indicator = f" [raw: {original_value:.4f}]" if abs(original_value - joint_value) > 0.0001 else ""
                        
                        if self.node:
                            self.node.get_logger().info(
                                f"🔄 {api_joint_name} -> {ros_joint_name}: {current_value:.4f} → {joint_value:.4f}{reverse_indicator}{smoothing_indicator}"
                            )
                    else:
                        if self.node:
                            self.node.get_logger().debug(
                                f"⏭️ {api_joint_name} -> {ros_joint_name}: No significant change ({current_value:.4f} ≈ {joint_value:.4f})"
                            )
                else:
                    if self.node:
                        self.node.get_logger().warning(
                            f"{api_joint_name} -> {ros_joint_name} not found in joint names"
                        )
        
        # Publish the updated joint state
        self.publisher.publish(updated_joint_state)
        if self.node:
            self.node.get_logger().debug(
                f"Published updated joint state to {self.publisher_topic}"
            )
    
    def start(self) -> None:
        """Start the adapter (initialize ROS2 node and begin operations)."""
        if not rclpy.ok():
            rclpy.init()
        
        self.node = Node("ros2_joint_state_adapter")
        
        self.node.declare_parameter("exponential_alpha", self.config.exponential_alpha)
        self.node.declare_parameter("history_length", self.config.history_length)
        self.node.declare_parameter("change_tolerance", self.config.change_tolerance)
        self.node.declare_parameter("joint_subscription_topic", self.subscription_topic)
        self.node.declare_parameter("joint_publisher_topic", self.publisher_topic)
        self.node.declare_parameter("joint_name_mapping", self.config.joint_name_mapping)
        
        self.config.exponential_alpha = self.node.get_parameter("exponential_alpha").value
        self.config.history_length = self.node.get_parameter("history_length").value
        self.config.change_tolerance = self.node.get_parameter("change_tolerance").value
        self.subscription_topic = self.node.get_parameter("joint_subscription_topic").value
        self.publisher_topic = self.node.get_parameter("joint_publisher_topic").value
        joint_name_mapping_str = self.node.get_parameter("joint_name_mapping").value
        
        self.joint_name_mapping = self._parse_joint_name_mapping(joint_name_mapping_str)
        
        if len(self.server_joint_state_history) == 0 or self.server_joint_state_history.maxlen != self.config.history_length:
            old_history = list(self.server_joint_state_history)
            self.server_joint_state_history = deque(maxlen=self.config.history_length)
            # Re-add history items up to new limit
            for item in old_history[-self.config.history_length:]:
                self.server_joint_state_history.append(item)
        
        self.node.get_logger().info(f"Joint name mappings: {self.joint_name_mapping}")
        
        self.subscription = self.node.create_subscription(
            JointState,
            self.subscription_topic,
            self._robot_joint_state_callback,
            1
        )
        
        self.publisher = self.node.create_publisher(
            JointState,
            self.publisher_topic,
            1
        )
        
        self.timer = self.node.create_timer(0.1, self._timer_callback)
        
        self.node.get_logger().info("ROS2 Joint State Adapter started")
    
    def stop(self) -> None:
        """Stop the adapter and clean up resources."""
        if self.timer:
            self.timer.cancel()
        
        if self.node:
            self.node.destroy_node()
        
        if rclpy.ok():
            rclpy.shutdown()


def main(args=None):
    """Main function to run the ROS2 Joint State Adapter."""
    parser = argparse.ArgumentParser(description="ROS2 Joint State Bridge")
    parser.add_argument(
        "--server-url",
        default="http://127.0.0.1:5000/get_joints",
        help="URL of the joint state server (default: http://127.0.0.1:5000/get_joints)",
    )
    parser.add_argument(
        "--exponential-alpha",
        type=float,
        default=0.05,
        help="Exponential smoothing factor (0.0-1.0, default: 0.05)",
    )
    parser.add_argument(
        "--history-length",
        type=int,
        default=15,
        help="Number of samples to keep in history (default: 15)",
    )
    parser.add_argument(
        "--change-tolerance",
        type=float,
        default=0.01,
        help="Minimum change threshold to update joint (default: 0.01)",
    )
    parser.add_argument(
        "--joint-subscription-topic",
        default="joint_states",
        help="ROS2 topic to subscribe to for joint states (default: joint_states)",
    )
    parser.add_argument(
        "--joint-publisher-topic",
        default="/joint_command",
        help="ROS2 topic to publish joint commands to (default: /joint_command)",
    )
    parser.add_argument(
        "--joint-name-mapping",
        default="",
        help="Comma-separated joint name mappings from API to ROS2 (format: api_name:ros_name:reverse,api_name2:ros_name2:reverse). Reverse is optional (true/false, default: false)",
    )

    # Parse known args to allow ROS2 args to pass through
    parsed_args, remaining_args = parser.parse_known_args()

    # Create adapter configuration with ROS2 messaging config
    messaging_config = {
        "subscription_topic": parsed_args.joint_subscription_topic,
        "publisher_topic": parsed_args.joint_publisher_topic
    }
    
    config = AdapterConfig(
        server_url=parsed_args.server_url,
        exponential_alpha=parsed_args.exponential_alpha,
        history_length=parsed_args.history_length,
        change_tolerance=parsed_args.change_tolerance,
        messaging_config=messaging_config,
        joint_name_mapping=parsed_args.joint_name_mapping
    )

    adapter = ROS2JointStateAdapter(config)
    
    try:
        rclpy.init(args=remaining_args)
        adapter.start()
        rclpy.spin(adapter.node)
    except KeyboardInterrupt:
        pass
    finally:
        adapter.stop()


if __name__ == "__main__":
    main()