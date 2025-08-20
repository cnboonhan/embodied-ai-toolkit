# Robot websocket adapter controller
# Subscribes to smoothed joint commands and publishes to arm/hand command topics
# python3 src/robot_websocket_adapter_ros2_controller.py --arm-command-topic /motion/control/arm_joint_command  --max-position-change-rad 1 --velocity-scale 0.1 --max-velocity-rad-s 0.3 --min-velocity-rad-s 0.02 --smoothing-alpha 0.4

import rclpy
import argparse
from rclpy.node import Node
from sensor_msgs.msg import JointState
import time


class JointStateReceiver(Node):
    def __init__(self, smoother_topic, arm_command_topic, hand_command_topic, arm_state_topic, hand_state_topic, 
                 max_position_change_rad, velocity_scale, min_velocity_rad_s, max_velocity_rad_s, smoothing_alpha):
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
        
        # Velocity generation parameters
        self.velocity_scale = velocity_scale
        self.min_velocity_rad_s = min_velocity_rad_s
        self.max_velocity_rad_s = max_velocity_rad_s
        self.previous_positions = {}  # Track previous positions for velocity calculation
        
        # Exponential smoothing parameters
        self.smoothing_alpha = smoothing_alpha
        self.smoothed_positions = {}  # Track smoothed positions for each joint
        
        # Subscribe to smoothed joint commands
        self.smoother_subscription = self.create_subscription(
            JointState,
            smoother_topic,
            self.smoother_callback,
            1,
        )
        
        # Subscribe to current joint states
        self.arm_state_subscription = self.create_subscription(
            JointState,
            arm_state_topic,
            self.arm_state_callback,
            1,
        )
        
        self.hand_state_subscription = self.create_subscription(
            JointState,
            hand_state_topic,
            self.hand_state_callback,
            1,
        )
        
        # Create publishers for arm and hand commands
        self.arm_command_publisher = self.create_publisher(
            JointState,
            arm_command_topic,
            1,
        )
        
        self.hand_command_publisher = self.create_publisher(
            JointState,
            hand_command_topic,
            10,
        )

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

    def limit_change(self, target_value, current_value, max_change, value_type, joint_name):
        """Limit the change to the maximum allowed value for any dimension"""
        if current_value is None:
            return target_value
        
        # Calculate the difference
        value_diff = target_value - current_value
        
        # Limit the change to the maximum allowed value
        if abs(value_diff) > max_change:
            if value_diff > 0:
                limited_value = current_value + max_change
            else:
                limited_value = current_value - max_change
            return limited_value
        else:
            return target_value



    def limit_position_change(self, target_position, joint_name, current_state_dict):
        """Limit the position change to the maximum allowed value for any dimension"""
        current_position = current_state_dict.get(joint_name)
        if current_position is None:
            return target_position
        
        # Calculate the difference
        position_diff = target_position - current_position
        
        # If change exceeds limit, use the previous value instead
        if abs(position_diff) > self.max_position_change_rad:
            return current_position
        else:
            return target_position
    
    def apply_exponential_smoothing(self, new_position, joint_name):
        """Apply exponential smoothing to joint position"""
        # Get previous smoothed position
        previous_smoothed = self.smoothed_positions.get(joint_name, new_position)
        
        # Apply exponential smoothing: smoothed = alpha * new + (1 - alpha) * previous
        smoothed_position = self.smoothing_alpha * new_position + (1 - self.smoothing_alpha) * previous_smoothed
        
        # Store smoothed position for next iteration
        self.smoothed_positions[joint_name] = smoothed_position
        
        return smoothed_position


    def generate_smooth_velocity(self, target_position, joint_name, current_state_dict, dt=0.033):
        """Generate smooth velocity command to prevent sudden stops"""
        current_position = current_state_dict.get(joint_name)
        if current_position is None:
            return 0.0
        
        # Calculate position error
        position_error = target_position - current_position
        
        # Calculate desired velocity based on position error
        desired_velocity = position_error * self.velocity_scale
        
        # Limit velocity to prevent sudden stops and excessive speed
        if abs(desired_velocity) < self.min_velocity_rad_s:
            # If velocity is too low but we're not at target, maintain minimum velocity
            if abs(position_error) > 0.001:  # Small threshold to prevent oscillation
                desired_velocity = self.min_velocity_rad_s if desired_velocity >= 0 else -self.min_velocity_rad_s
            else:
                # Very close to target, reduce velocity gradually
                desired_velocity = desired_velocity * 0.1
        
        # Apply maximum velocity limit
        if abs(desired_velocity) > self.max_velocity_rad_s:
            desired_velocity = self.max_velocity_rad_s if desired_velocity > 0 else -self.max_velocity_rad_s
        
        return desired_velocity


    def smoother_callback(self, msg):
        
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
                    # Apply exponential smoothing first
                    smoothed_position = self.apply_exponential_smoothing(msg.position[i], joint_name)
                    
                    # Then apply position change limiting
                    limited_position = self.limit_position_change(smoothed_position, joint_name, self.latest_arm_state['position'])
                    arm_positions.append(limited_position)
                    
                    # Generate smooth velocity command to prevent sudden stops
                    smooth_velocity = self.generate_smooth_velocity(limited_position, joint_name, self.latest_arm_state['position'])
                    arm_velocities.append(smooth_velocity)
                
                # Add effort without limiting (if provided)
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
            
            # Set timestamp using Python time
            current_time_sec = time.time()
            current_time_nanosec = int((current_time_sec % 1) * 1e9)
            current_time_sec_int = int(current_time_sec)
            
            from builtin_interfaces.msg import Time
            timestamp = Time()
            timestamp.sec = current_time_sec_int
            timestamp.nanosec = current_time_nanosec
            arm_msg.header.stamp = timestamp
            
            arm_msg.name = arm_joints
            
            if arm_positions:
                arm_msg.position = arm_positions
            if arm_velocities:
                arm_msg.velocity = arm_velocities
            if arm_efforts:
                arm_msg.effort = arm_efforts
                
            self.arm_command_publisher.publish(arm_msg)
        
        # Publish hand joint commands
        if hand_joints:
            hand_msg = JointState()
            
            # Set timestamp using Python time
            current_time_sec = time.time()
            current_time_nanosec = int((current_time_sec % 1) * 1e9)
            current_time_sec_int = int(current_time_sec)
            
            from builtin_interfaces.msg import Time
            timestamp = Time()
            timestamp.sec = current_time_sec_int
            timestamp.nanosec = current_time_nanosec
            hand_msg.header.stamp = timestamp
            
            hand_msg.name = hand_joints
            
            if hand_positions:
                hand_msg.position = hand_positions
            if hand_velocities:
                hand_msg.velocity = hand_velocities
            if hand_efforts:
                hand_msg.effort = hand_efforts
                
            self.hand_command_publisher.publish(hand_msg)


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
        default=0.01,
        help='Maximum allowed position change in radians per command (default: 0.01)'
    )
    parser.add_argument(
        '--velocity-scale',
        type=float,
        default=0.5,
        help='Velocity scaling factor for smooth motion (default: 0.5)'
    )
    parser.add_argument(
        '--min-velocity-rad-s',
        type=float,
        default=0.01,
        help='Minimum velocity to prevent sudden stops (default: 0.01)'
    )
    parser.add_argument(
        '--max-velocity-rad-s',
        type=float,
        default=0.5,
        help='Maximum velocity limit (default: 0.5)'
    )
    parser.add_argument(
        '--smoothing-alpha',
        type=float,
        default=0.3,
        help='Exponential smoothing alpha (0.1-0.9, lower=smother, default: 0.3)'
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
        cli_args.max_position_change_rad,
        cli_args.velocity_scale,
        cli_args.min_velocity_rad_s,
        cli_args.max_velocity_rad_s,
        cli_args.smoothing_alpha
    )
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()