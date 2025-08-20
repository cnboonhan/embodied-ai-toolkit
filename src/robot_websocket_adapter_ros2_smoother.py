# websocat --udp-server-buffer-size  1000  --no-close ws://localhost:5000/ws/update_joints | python3 src/robot_websocket_adapter_ros2_smoother.py  --max-output-rate 70

import sys
import time
import json
import argparse
from collections import defaultdict, deque

# ROS imports
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState

def parse_arguments():
    """Parse command line arguments"""
    parser = argparse.ArgumentParser(description='Robot websocket adapter - publishes raw joint state to ROS topic')
    parser.add_argument(
        '--joint-state-topic',
        type=str,
        default='/joint_command_smoothed',
        help='ROS topic for publishing joint state (default: /joint_command_smoothed)'
    )
    parser.add_argument(
        '--max-output-rate',
        type=float,
        default=80.0,
        help='Maximum output rate in Hz (default: 20.0)'
    )
    return parser.parse_args()

def main():
    """Main function"""
    args = parse_arguments()
    
    rclpy.init()
    node = Node('robot_websocket_adapter')
    
    publisher = node.create_publisher(JointState, args.joint_state_topic, 10)
      
    MAX_OUTPUT_RATE = args.max_output_rate
    OUTPUT_INTERVAL = 1.0 / MAX_OUTPUT_RATE
    
    # Rate limiting variables
    last_output_time = 0.0
    pending_joint_values = None
    output_count = 0
    
    def execute_control_command(joint_values):
        joints = []
        positions = []
        
        try:
            for joint_name, value in joint_values.items():
                if value is not None:
                    joints.append(joint_name)
                    positions.append(value)
                    
            if joints:  # Only publish if we have valid joints
                msg = JointState()
                msg.header.stamp = node.get_clock().now().to_msg()
                msg.name = joints
                msg.position = positions
                publisher.publish(msg)
                return True

        except ValueError as e:
            print(f"Error processing joints: {e}")
            print("Skipping entire command - no joint commands will be published")
        
        return False
    
    print(f"Starting robot websocket adapter with:")
    print(f"  Joint state topic: {args.joint_state_topic}")
    print(f"  Max output rate: {MAX_OUTPUT_RATE} Hz (interval: {OUTPUT_INTERVAL*1000:.1f} ms)")
    
    try:
        buff = ''
        while True:
            current_time = time.time()
            
            # Check if it's time to publish
            if current_time - last_output_time >= OUTPUT_INTERVAL and pending_joint_values is not None:
                if execute_control_command(pending_joint_values):
                    output_count += 1
                    if output_count % 50 == 0:  # Log every 50 publications
                        print(f"📊 Published {output_count} messages at {MAX_OUTPUT_RATE} Hz")
                last_output_time = current_time
            
            char = sys.stdin.read(1)
            if not char:
                break
            
            buff += char
            
            if buff.endswith('\n'):
                message = buff.rstrip('\n')
                
                try:
                    parsed = json.loads(message)
                    print(f"Received websocket message: {parsed}")
                    
                    # Extract joint values directly without any smoothing
                    joint_values = {}
                    for joint_name, joint_data in parsed.items():
                        if isinstance(joint_data, dict) and 'value' in joint_data:
                            joint_values[joint_name] = joint_data['value']
                    
                    # Store for rate-limited publishing
                    pending_joint_values = joint_values
                    
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
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()