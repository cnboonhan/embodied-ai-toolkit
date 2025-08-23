#!/usr/bin/env python3
"""
Streamer.py - A Python script that executes the stream joints command and parses continuous JSON data.

Usage:
    python streamer.py
    echo '{"update_frequency": 100.0}' | python streamer.py
"""

import json
import subprocess
import sys
import time
from typing import Dict, Any


def stream_joint_data(update_frequency: float = 100.0) -> None:
    """
    Execute the StreamJointData command and parse the continuous JSON stream.
    
    Args:
        update_frequency: Frequency of joint data updates in Hz
    """
    # Construct the grpcurl command
    command = [
        "grpcurl",
        "-plaintext",
        "-format", "json",
        "-d", json.dumps({"update_frequency": update_frequency}),
        "localhost:5000",
        "rosbot_api.RobotApiService/StreamJointData"
    ]
    
    try:
        print(f"Starting joint data stream with frequency: {update_frequency} Hz", file=sys.stderr)
        print("Press Ctrl+C to stop streaming", file=sys.stderr)
        print("-" * 50, file=sys.stderr)
        
        # Execute the streaming command
        process = subprocess.Popen(
            command,
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            text=True,
            bufsize=1,
            universal_newlines=True
        )
        
        # Read and parse the continuous stream
        buffer = ""
        brace_count = 0
        
        for line in process.stdout:
            line = line.strip()
            if line:
                buffer += line
                # Count braces to detect complete JSON objects
                brace_count += line.count('{') - line.count('}')
                
                # If we have a complete JSON object (balanced braces)
                if brace_count == 0 and buffer:
                    try:
                        # Parse the complete JSON object
                        json_data = json.loads(buffer)
                        
                        # Pretty print the JSON data
                        print(json.dumps(json_data, indent=2))
                        
                        # Add a separator for readability
                        print("-" * 30)
                        
                        # Reset buffer for next JSON object
                        buffer = ""
                        
                    except json.JSONDecodeError as e:
                        # If it's not valid JSON, print as-is
                        print(f"Non-JSON data received: {buffer}")
                        print(f"JSON decode error: {e}")
                        buffer = ""
                        brace_count = 0
                    
    except KeyboardInterrupt:
        print("\nStreaming interrupted by user", file=sys.stderr)
        process.terminate()
        process.wait()
    except Exception as e:
        print(f"Error in streaming: {e}", file=sys.stderr)
        if 'process' in locals():
            process.terminate()


def main():
    """Main function to handle stream joints command."""
    # Check if frequency is provided via stdin
    if not sys.stdin.isatty():
        try:
            # Read frequency from stdin
            stdin_data = sys.stdin.read().strip()
            if stdin_data:
                config = json.loads(stdin_data)
                frequency = config.get("update_frequency", 100.0)
            else:
                frequency = 100.0
        except json.JSONDecodeError:
            print("Invalid JSON in stdin, using default frequency", file=sys.stderr)
            frequency = 100.0
    else:
        # Use default frequency
        frequency = 100.0
    
    # Start streaming joint data
    stream_joint_data(frequency)


if __name__ == "__main__":
    main()
