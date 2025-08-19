# Robot websocket adapter controller
# Subscribes to smoothed joint commands and publishes to arm/hand command topics

import rclpy
import argparse
from rclpy.node import Node
from sensor_msgs.msg import JointState


class JointStateReceiver(Node):
    def __init__(self, smoother_topic, arm_command_topic, hand_command_topic, arm_state_topic, hand_state_topic, max_angle_change_rad):
        super().__init__("robot_websocket_adapter_ros2_controller")
        
        # Initialize joint state variables
        self.latest_arm_state = None
        self.latest_hand_state = None
        self.max_angle_change_rad = max_angle_change_rad
        
        # Subscribe to smoothed joint commands
        self.smoother_subscription = self.create_subscription(
            JointState,
            smoother_topic,
            self.smoother_callback,
            10,
        )
        self.get_logger().info(f"Subscribed to {smoother_topic}")
        
        # Subscribe to current joint states
        self.arm_state_subscription = self.create_subscription(
            JointState,
            arm_state_topic,
            self.arm_state_callback,
            10,
        )
        self.get_logger().info(f"Subscribed to {arm_state_topic}")
        
        self.hand_state_subscription = self.create_subscription(
            JointState,
            hand_state_topic,
            self.hand_state_callback,
            10,
        )
        self.get_logger().info(f"Subscribed to {hand_state_topic}")
        
        # Create publishers for arm and hand commands
        self.arm_command_publisher = self.create_publisher(
            JointState,
            arm_command_topic,
            10,
        )
        self.get_logger().info(f"Publisher created for {arm_command_topic}")
        
        self.hand_command_publisher = self.create_publisher(
            JointState,
            hand_command_topic,
            10,
        )
        self.get_logger().info(f"Publisher created for {hand_command_topic}")

    def arm_state_callback(self, msg):
        """Callback function for arm joint state messages"""
        self.latest_arm_state = msg
        self.get_logger().debug(f"Updated arm state: {msg.name} -> {msg.position}")

    def hand_state_callback(self, msg):
        """Callback function for hand joint state messages"""
        self.latest_hand_state = msg
        self.get_logger().debug(f"Updated hand state: {msg.name} -> {msg.position}")

    def limit_angle_change(self, target_position, joint_name, current_state_msg):
        """Limit the angle change to the maximum allowed value"""
        if current_state_msg is None:
            self.get_logger().warning(f"No current state available for {joint_name}, using target position")
            return target_position
        
        # Find the current position for this joint
        current_position = None
        for i, name in enumerate(current_state_msg.name):
            if name == joint_name:
                current_position = current_state_msg.position[i]
                break
        
        if current_position is None:
            self.get_logger().warning(f"Joint {joint_name} not found in current state, using target position")
            return target_position
        
        # Calculate the angle difference
        angle_diff = target_position - current_position
        
        # Limit the angle change to the maximum allowed value
        if abs(angle_diff) > self.max_angle_change_rad:
            if angle_diff > 0:
                limited_position = current_position + self.max_angle_change_rad
            else:
                limited_position = current_position - self.max_angle_change_rad
            self.get_logger().info(f"Limited {joint_name} angle change: {angle_diff:.4f} -> {self.max_angle_change_rad:.4f} rad")
            return limited_position
        else:
            return target_position

    def smoother_callback(self, msg):
        """Callback function for smoothed joint state messages"""
        self.get_logger().info(f"Received smoothed joint state: {msg.name} -> {msg.position}")
        
        # Separate arm and hand joints
        arm_joints = []
        arm_positions = []
        hand_joints = []
        hand_positions = []
        
        for i, joint_name in enumerate(msg.name):
            # Classify joints based on naming convention:
            # - arm_joints: prefixed with idx13 to idx26
            # - hand_joints: prefixed with left_ or right_
            if joint_name.startswith(('idx13', 'idx14', 'idx15', 'idx16', 'idx17', 'idx18', 
                                    'idx19', 'idx20', 'idx21', 'idx22', 'idx23', 'idx24', 
                                    'idx25', 'idx26')):
                arm_joints.append(joint_name)
                limited_position = self.limit_angle_change(msg.position[i], joint_name, self.latest_arm_state)
                arm_positions.append(limited_position)
            elif joint_name.startswith(('left_', 'right_')):
                hand_joints.append(joint_name)
                limited_position = self.limit_angle_change(msg.position[i], joint_name, self.latest_hand_state)
                hand_positions.append(limited_position)
        
        # Publish arm joint commands
        if arm_joints:
            arm_msg = JointState()
            arm_msg.header.stamp = self.get_clock().now().to_msg()
            arm_msg.name = arm_joints
            arm_msg.position = arm_positions
            self.arm_command_publisher.publish(arm_msg)
            self.get_logger().info(f"Published arm command: {arm_joints} -> {arm_positions}")
        
        # Publish hand joint commands
        if hand_joints:
            hand_msg = JointState()
            hand_msg.header.stamp = self.get_clock().now().to_msg()
            hand_msg.name = hand_joints
            hand_msg.position = hand_positions
            self.hand_command_publisher.publish(hand_msg)
            self.get_logger().info(f"Published hand command: {hand_joints} -> {hand_positions}")




def parse_arguments():
    """Parse command line arguments"""
    parser = argparse.ArgumentParser(description='Robot websocket adapter controller')
    parser.add_argument(
        '--smoother-topic',
        type=str,
        default='/joint_command_smoothed',
        help='ROS topic for smoothed joint commands (default: /joint_command_smoothed)'
    )
    parser.add_argument(
        '--arm-command-topic',
        type=str,
        default='/arm_joint_command',
        help='ROS topic for arm joint commands (default: arm_joint_command)'
    )
    parser.add_argument(
        '--hand-command-topic',
        type=str,
        default='/hand_joint_command',
        help='ROS topic for hand joint commands (default: /motion/control/hand_joint_command)'
    )
    parser.add_argument(
        '--arm-state-topic',
        type=str,
        default='/motion/control/arm_joint_state',
        help='ROS topic for current arm joint states (default: /motion/control/arm_joint_state)'
    )
    parser.add_argument(
        '--hand-state-topic',
        type=str,
        default='/motion/control/hand_joint_state',
        help='ROS topic for current hand joint states (default: /motion/control/hand_joint_state)'
    )
    parser.add_argument(
        '--max-angle-change-rad',
        type=float,
        default=0.01,
        help='Maximum allowed angle change in radians per command (default: 0.01)'
    )
    return parser.parse_args()

def main(args=None):
    # Parse command line arguments
    cli_args = parse_arguments()
    
    rclpy.init(args=args)
    node = JointStateReceiver(
        cli_args.smoother_topic, 
        cli_args.arm_command_topic, 
        cli_args.hand_command_topic,
        cli_args.arm_state_topic,
        cli_args.hand_state_topic,
        cli_args.max_angle_change_rad
    )
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()