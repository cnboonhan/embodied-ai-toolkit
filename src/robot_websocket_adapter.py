# websocat --no-close ws://localhost:5000/ws/update_joints | python3 src/robot_websocket_adapter.py

import sys
import time
import json
import argparse
from collections import defaultdict, deque
from ccma import CCMA
import numpy as np

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

if __name__ == "__main__":
    main()