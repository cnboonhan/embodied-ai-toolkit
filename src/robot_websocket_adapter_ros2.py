# websocat --no-close ws://localhost:5000/ws/update_joints | python3 src/robot_websocket_adapter_ros2.py

import sys
import time
import json
import argparse
from collections import defaultdict, deque
from ccma import CCMA
import numpy as np

# ROS imports
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState

def parse_arguments():
    """Parse command line arguments"""
    parser = argparse.ArgumentParser(description='Robot websocket adapter with joint history tracking and CCMA smoothing')
    parser.add_argument(
        '--window-size', 
        '-w', 
        type=int, 
        default=10,
        help='Number of values to keep in history for each joint (default: 10)'
    )
    parser.add_argument(
        '--ccma-w-ma',
        type=int,
        default=5,
        help='CCMA moving average width parameter (default: 5)'
    )
    parser.add_argument(
        '--ccma-w-cc',
        type=int,
        default=3,
        help='CCMA curvature correction width parameter (default: 3)'
    )
    parser.add_argument(
        '--ccma-distrib',
        choices=['uniform', 'normal', 'pascal', 'hanning'],
        default='pascal',
        help='CCMA distribution type (default: pascal)'
    )
    parser.add_argument(
        '--ccma-rho-ma',
        type=float,
        default=0.95,
        help='CCMA rho parameter for moving average (default: 0.95)'
    )
    parser.add_argument(
        '--ccma-rho-cc',
        type=float,
        default=0.95,
        help='CCMA rho parameter for curvature correction (default: 0.95)'
    )
    parser.add_argument(
        '--arm-topic',
        type=str,
        default='/arm_joint_command',
        help='ROS topic for arm joint commands (default: /arm_joint_command)'
    )
    parser.add_argument(
        '--hand-topic',
        type=str,
        default='/hand_joint_command',
        help='ROS topic for hand joint commands (default: /hand_joint_command)'
    )
    return parser.parse_args()

def generate_smoothed_values(joint_histories, ccma_params):
    smoothed_values = {}
    
    ccma_filter = CCMA(
        w_ma=ccma_params['w_ma'],
        w_cc=ccma_params['w_cc'],
        distrib=ccma_params['distrib'],
        rho_ma=ccma_params['rho_ma'],
        rho_cc=ccma_params['rho_cc']
    )
    
    for joint_name, history in joint_histories.items():
        if len(history) > 0:
            history_list = list(history)
            path_points = np.array([[i, val, 0] for i, val in enumerate(history_list)])
            
            try:
                smoothed_path = ccma_filter.filter(path_points, mode="padding", cc_mode=True)
                smoothed_values[joint_name] = smoothed_path[-1, 1] 
            except Exception as e:
                print(f"CCMA smoothing failed for {joint_name}: {e}")
                smoothed_values[joint_name] = history_list[-1]
        else:
            smoothed_values[joint_name] = None
    
    return smoothed_values

def main():
    """Main function"""
    args = parse_arguments()
    
    # Initialize ROS
    rclpy.init()
    node = Node('robot_websocket_adapter')
    
    # Create publishers
    arm_publisher = node.create_publisher(JointState, args.arm_topic, 10)
    hand_publisher = node.create_publisher(JointState, args.hand_topic, 10)
    
    HISTORY_WINDOW_SIZE = args.window_size
    
    # CCMA parameters
    ccma_params = {
        'w_ma': args.ccma_w_ma,
        'w_cc': args.ccma_w_cc,
        'distrib': args.ccma_distrib,
        'rho_ma': args.ccma_rho_ma,
        'rho_cc': args.ccma_rho_cc
    }
    
    joint_history = defaultdict(lambda: deque(maxlen=HISTORY_WINDOW_SIZE))
    
    def add_to_joint_history(joint_name, value):
        joint_history[joint_name].append(value)
    
    def get_joint_history(joint_name):
        return joint_history[joint_name]
    
    def execute_control_command(smoothed_values):
        print(f"Smoothed Values: {smoothed_values}")
        
        # Separate arm and hand joints (you may need to adjust these based on your robot's joint names)
        arm_joints = []
        arm_positions = []
        hand_joints = []
        hand_positions = []
        
        for joint_name, value in smoothed_values.items():
            if value is not None:
                # Classify joints based on naming convention:
                # - arm_joints: prefixed with idx13 to idx26
                # - hand_joints: prefixed with left_ or right_
                if joint_name.startswith(('idx13', 'idx14', 'idx15', 'idx16', 'idx17', 'idx18', 
                                        'idx19', 'idx20', 'idx21', 'idx22', 'idx23', 'idx24', 
                                        'idx25', 'idx26')):
                    arm_joints.append(joint_name)
                    arm_positions.append(value)
                elif joint_name.startswith(('left_', 'right_')):
                    hand_joints.append(joint_name)
                    hand_positions.append(value)
        
        # Publish arm joint commands
        if arm_joints:
            arm_msg = JointState()
            arm_msg.header.stamp = node.get_clock().now().to_msg()
            arm_msg.name = arm_joints
            arm_msg.position = arm_positions
            arm_publisher.publish(arm_msg)
            print(f"Published arm command: {arm_joints} -> {arm_positions}")
        
        # Publish hand joint commands
        if hand_joints:
            hand_msg = JointState()
            hand_msg.header.stamp = node.get_clock().now().to_msg()
            hand_msg.name = hand_joints
            hand_msg.position = hand_positions
            hand_publisher.publish(hand_msg)
            print(f"Published hand command: {hand_joints} -> {hand_positions}")
    
    def print_smoothed_values(smoothed_values):
        print("Smoothed Values (CCMA):")
        for joint_name, value in smoothed_values.items():
            if value is not None:
                print(f"  {joint_name}: {value:.4f}")
        print("-" * 50)
    
    print(f"Starting robot websocket adapter with:")
    print(f"  History window size: {HISTORY_WINDOW_SIZE}")
    print(f"  CCMA parameters: w_ma={ccma_params['w_ma']}, w_cc={ccma_params['w_cc']}, distrib={ccma_params['distrib']}")
    print(f"  CCMA rho: rho_ma={ccma_params['rho_ma']}, rho_cc={ccma_params['rho_cc']}")
    print(f"  Arm topic: {args.arm_topic}")
    print(f"  Hand topic: {args.hand_topic}")
    
    try:
        buff = ''
        while True:
            char = sys.stdin.read(1)
            if not char:
                break
            
            buff += char
            
            if buff.endswith('\n'):
                message = buff.rstrip('\n')
                print(f"Received websocket message: {message}")
                
                try:
                    parsed = json.loads(message)
                    
                    for joint_name, joint_data in parsed.items():
                        if isinstance(joint_data, dict) and 'value' in joint_data:
                            add_to_joint_history(joint_name, joint_data['value'])
                    
                    # Check if all joints have enough history for CCMA filtering
                    min_points_needed = ccma_params['w_ma'] + ccma_params['w_cc'] + 1
                    all_joints_ready = all(len(history) >= min_points_needed for history in joint_history.values() if len(history) > 0)
                    
                    if all_joints_ready:
                        smoothed_values = generate_smoothed_values(joint_history, ccma_params)
                        execute_control_command(smoothed_values)
                    else:
                        # Print current history lengths for debugging
                        print("Waiting for all joints to reach minimum history length...")
                        for joint_name, history in joint_history.items():
                            if len(history) > 0:
                                print(f"  {joint_name}: {len(history)}/{min_points_needed} points")
                except json.JSONDecodeError:
                    pass
                
                buff = ''
                
    except KeyboardInterrupt:
        print("\nInterrupted by user")
        sys.stdout.flush()
    except Exception as e:
        print(f"Error: {e}")
        sys.stdout.flush()
    finally:
        # Clean up ROS
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()