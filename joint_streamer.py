#!/usr/bin/env python3
"""
Streamer.py - A Python script that executes the stream joints command and parses continuous JSON data. Only requies grpcurl binary dependency

Usage:
    python joint_streamer.py --config_path config.json
    python joint_streamer.py --config_path config.json --update_frequency 100.0
"""

import json
import subprocess
import sys
import time
from typing import Dict, Any, Optional
from pathlib import Path
import tyro
from src.utils import load_config, Config


def stream_joint_data(config: Config, update_frequency: float = 100.0, api_port: Optional[int] = None) -> None:
    """
    Execute the StreamJointData command and parse the continuous JSON stream.
    
    Args:
        config_data: Configuration data loaded from config file
        update_frequency: Frequency of joint data updates in Hz
        api_port: API port to connect to (overrides config if provided)
    """

    
    # Construct the grpcurl command
    command = [
        "grpcurl",
        "-plaintext",
        "-format", "json",
        "-d", json.dumps({"update_frequency": update_frequency}),
        f"localhost:{config.api_port}",
        "rosbot_api.RobotApiService/StreamJointData"
    ]
    
    try:
        print(f"Starting joint data stream with frequency: {update_frequency} Hz", file=sys.stderr)
        print(f"Connecting to API on port: {config.api_port}", file=sys.stderr)
        print(f"Using label: {config.label}", file=sys.stderr)
        print(f"Project name: {config.project_name}", file=sys.stderr)
        print(f"Episode name: {config.epsisode_name}", file=sys.stderr)
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
                        
                        # Joint states are printed to console
                        
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


def main(config_path: Path, update_frequency: float = 100.0, api_port: Optional[int] = None):
    """Main function to handle stream joints command."""
    # Load configuration from file
    try:
        config = load_config(config_path)
        print(f"Loaded configuration from: {config_path}", file=sys.stderr)
    except Exception as e:
        print(f"Error loading config file: {e}", file=sys.stderr)
        sys.exit(1)
    
    # Start streaming joint data
    stream_joint_data(config, update_frequency, api_port)


if __name__ == "__main__":
    tyro.cli(main)
