# Robot websocket adapter controller
# Subscribes to smoothed joint commands and publishes to arm/hand command topics
# python3 src/robot_websocket_adapter_ros2_controller.py --arm-command-topic /motion/control/arm_joint_command

import rclpy
import argparse
from rclpy.node import Node
from sensor_msgs.msg import JointState


class JointStateReceiver(Node):
    def __init__(self, smoother_topic, arm_command_topic, hand_command_topic, arm_state_topic, hand_state_topic, 
                 max_position_change_rad):
        super().__init__("robot_websocket_adapter_ros2_controller")
        
        # Initialize joint state variables - track position, velocity, and effort
        self.latest_arm_state = {
            'position': {},
            'velocity': {},
            'effort': {}
        }
        self.latest_hand_state = {
            'position': {},
            'velocity': {},
            'effort': {}
        }
        
        # Change limiting parameters
        self.max_position_change_rad = max_position_change_rad
        
        # Subscribe to smoothed joint commands
        self.smoother_subscription = self.create_subscription(
            JointState,
            smoother_topic,
            self.smoother_callback,
            1,
        )
        self.get_logger().info(f"Subscribed to {smoother_topic}")
        
        # Subscribe to current joint states
        self.arm_state_subscription = self.create_subscription(
            JointState,
            arm_state_topic,
            self.arm_state_callback,
            1,
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
        # Update position, velocity, and effort for each joint
        for i, name in enumerate(msg.name):
            # Update position
            if i < len(msg.position):
                self.latest_arm_state['position'][name] = msg.position[i]
            
            # Update velocity
            if i < len(msg.velocity):
                self.latest_arm_state['velocity'][name] = msg.velocity[i]
            
            # Update effort
            if i < len(msg.effort):
                self.latest_arm_state['effort'][name] = msg.effort[i]
        
        self.get_logger().debug(f"Updated arm state: {self.latest_arm_state}")

    def hand_state_callback(self, msg):
        """Callback function for hand joint state messages"""
        # Update position, velocity, and effort for each joint
        for i, name in enumerate(msg.name):
            # Update position
            if i < len(msg.position):
                self.latest_hand_state['position'][name] = msg.position[i]
            
            # Update velocity
            if i < len(msg.velocity):
                self.latest_hand_state['velocity'][name] = msg.velocity[i]
            
            # Update effort
            if i < len(msg.effort):
                self.latest_hand_state['effort'][name] = msg.effort[i]
        
        self.get_logger().debug(f"Updated hand state: {self.latest_hand_state}")

    def limit_change(self, target_value, current_value, max_change, value_type, joint_name):
        """Limit the change to the maximum allowed value for any dimension"""
        if current_value is None:
            self.get_logger().warning(f"No current {value_type} available for {joint_name}, using target value")
            return target_value
        
        # Calculate the difference
        value_diff = target_value - current_value
        
        # Limit the change to the maximum allowed value
        if abs(value_diff) > max_change:
            if value_diff > 0:
                limited_value = current_value + max_change
            else:
                limited_value = current_value - max_change
            self.get_logger().info(f"Limited {joint_name} {value_type} change: {value_diff:.4f} -> {max_change:.4f}")
            return limited_value
        else:
            return target_value



    def limit_position_change(self, target_position, joint_name, current_state_dict):
        """Limit the position change to the maximum allowed value"""
        current_position = current_state_dict.get(joint_name)
        return self.limit_change(target_position, current_position, self.max_position_change_rad, "position", joint_name)



    def smoother_callback(self, msg):
        """Callback function for smoothed joint state messages"""
        self.get_logger().info(f"Received smoothed joint state: {msg.name}")
        if msg.position:
            self.get_logger().info(f"  Positions: {msg.position}")
        if msg.velocity:
            self.get_logger().info(f"  Velocities: {msg.velocity}")
        if msg.effort:
            self.get_logger().info(f"  Efforts: {msg.effort}")
        
        # Separate arm and hand joints
        arm_joints = []
        arm_positions = []
        arm_velocities = []
        arm_efforts = []
        
        hand_joints = []
        hand_positions = []
        hand_velocities = []
        hand_efforts = []
        
        for i, joint_name in enumerate(msg.name):
            # Classify joints based on naming convention:
            # - arm_joints: prefixed with idx13 to idx26
            # - hand_joints: prefixed with left_ or right_
            if joint_name.startswith(('idx13', 'idx14', 'idx15', 'idx16', 'idx17', 'idx18', 
                                    'idx19', 'idx20', 'idx21', 'idx22', 'idx23', 'idx24', 
                                    'idx25', 'idx26')):
                arm_joints.append(joint_name)
                
                # Limit position change
                if i < len(msg.position):
                    limited_position = self.limit_position_change(msg.position[i], joint_name, self.latest_arm_state['position'])
                    arm_positions.append(limited_position)
                
                # Add velocity and effort without limiting
                if i < len(msg.velocity):
                    arm_velocities.append(msg.velocity[i])
                
                if i < len(msg.effort):
                    arm_efforts.append(msg.effort[i])
                    
            # elif joint_name.startswith(('left_', 'right_')):
            #     hand_joints.append(joint_name)
            #     
            #     # Limit position change
            #     if i < len(msg.position):
            #         limited_position = self.limit_position_change(msg.position[i], joint_name, self.latest_hand_state['position'])
            #         hand_positions.append(limited_position)
            #     
            #     # Add velocity and effort without limiting
            #     if i < len(msg.velocity):
            #         hand_velocities.append(msg.velocity[i])
            #     
            #     if i < len(msg.effort):
            #         hand_efforts.append(msg.effort[i])
        
        # Publish arm joint commands
        if arm_joints:
            arm_msg = JointState()
            arm_msg.header.stamp = self.get_clock().now().to_msg()
            arm_msg.name = arm_joints
            
            if arm_positions:
                arm_msg.position = arm_positions
            if arm_velocities:
                arm_msg.velocity = arm_velocities
            if arm_efforts:
                arm_msg.effort = arm_efforts
                
            self.arm_command_publisher.publish(arm_msg)
            self.get_logger().info(f"Published arm command: {arm_joints}")
            if arm_positions:
                self.get_logger().info(f"  Positions: {arm_positions}")
            if arm_velocities:
                self.get_logger().info(f"  Velocities: {arm_velocities}")
            if arm_efforts:
                self.get_logger().info(f"  Efforts: {arm_efforts}")
        
        # Publish hand joint commands
        if hand_joints:
            hand_msg = JointState()
            hand_msg.header.stamp = self.get_clock().now().to_msg()
            hand_msg.name = hand_joints
            
            if hand_positions:
                hand_msg.position = hand_positions
            if hand_velocities:
                hand_msg.velocity = hand_velocities
            if hand_efforts:
                hand_msg.effort = hand_efforts
                
            self.hand_command_publisher.publish(hand_msg)
            self.get_logger().info(f"Published hand command: {hand_joints}")
            if hand_positions:
                self.get_logger().info(f"  Positions: {hand_positions}")
            if hand_velocities:
                self.get_logger().info(f"  Velocities: {hand_velocities}")
            if hand_efforts:
                self.get_logger().info(f"  Efforts: {hand_efforts}")


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
        '--max-position-change-rad',
        type=float,
        default=0.005,  # Reduced from 0.01 to 0.005 for smoother motion
        help='Maximum allowed position change in radians per command (default: 0.005)'
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
        cli_args.max_position_change_rad
    )
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()