#!/usr/bin/env python3
"""
Robot Rerun Visualizer

This script interfaces with the robot Action Controller and State Visualizer servers
and pushes the data to Rerun for real-time visualization. It follows the pattern
described in the README.md where you have:
1. An action controller server (port 5000)
2. A state visualizer server (port 5001)
3. This script that bridges them and visualizes data in Rerun

Usage:
    python robot_rerun_visualizer.py --action-server http://localhost:5000 --state-server http://localhost:5001
"""

import argparse
import json
import time
import requests

import sys
import os
from typing import Dict, Any, List
import logging
from dataclasses import dataclass
from pathlib import Path

import rerun as rr


@dataclass
class RobotData:
    """Container for robot data in the specified format."""

    observation_state: List[float]
    action: List[float]
    timestamp: float
    annotation_human_action_task_description: int
    task_index: int
    annotation_human_validity: int
    episode_index: int
    frame_index: int
    index: int
    next_reward: float
    next_done: bool


class RobotRerunVisualizer:
    """Visualizes robot data in Rerun by interfacing with robot servers."""

    def __init__(
        self,
        action_server_url: str,
        state_server_url: str,
        rerun_endpoint: str = "rerun+http://localhost:9876/proxy",
        fps: int = 30,
        task_description: str = "Robot Control Task",
    ):
        """
        Initialize the robot rerun visualizer.

        Args:
            action_server_url: URL of the action server (e.g., http://localhost:5000)
            state_server_url: URL of the state visualizer server (e.g., http://localhost:5001)
            rerun_port: Port for Rerun server
            fps: Frames per second for data collection
            task_description: Description of the current task
        """
        self.action_server_url = action_server_url.rstrip("/")
        self.state_server_url = state_server_url.rstrip("/")
        self.rerun_endpoint = rerun_endpoint
        self.fps = fps
        self.task_description = task_description

        # Setup logging
        logging.basicConfig(level=logging.INFO)
        self.logger = logging.getLogger(__name__)

        # Data collection state
        self.is_collecting = False
        self.frame_index = 0
        self.start_time = None
        self.joint_names = []

        # Initialize Rerun
        self._init_rerun()

    def _init_rerun(self):
        """Initialize Rerun for visualization."""
        try:
            # Initialize Rerun
            rr.init("robot_visualizer", spawn=False)
            rr.connect_grpc(self.rerun_endpoint, flush_timeout_sec=5.0)

            self.logger.info(f"Rerun initialized with endpoint: {self.rerun_endpoint}")
        except Exception as e:
            self.logger.error(f"Failed to initialize Rerun: {e}")
            raise

    def get_joint_data(self, server_url: str) -> Dict[str, Any]:
        """Get joint data from a server."""
        try:
            response = requests.get(f"{server_url}/get_joints", timeout=1)
            if response.status_code == 200:
                return response.json()
            else:
                self.logger.warning(
                    f"Failed to get joints from {server_url}: {response.status_code}"
                )
                return {}
        except requests.exceptions.RequestException as e:
            self.logger.warning(f"Request failed for {server_url}: {e}")
            return {}

    def create_robot_data(
        self,
        action_joints: Dict[str, Any],
        state_joints: Dict[str, Any],
        current_time: float,
    ) -> RobotData:
        """Create robot data in the specified format."""
        # Extract joint values in consistent order
        action_values = [
            action_joints.get(name, {}).get("value", 0.0) for name in self.joint_names
        ]
        state_values = [
            state_joints.get(name, {}).get("value", 0.0) for name in self.joint_names
        ]

        return RobotData(
            observation_state=state_values,
            action=action_values,
            timestamp=current_time,
            annotation_human_action_task_description=0,  # Default task description index
            task_index=0,  # Default task index
            annotation_human_validity=1,  # Valid annotation
            episode_index=0,
            frame_index=self.frame_index,
            index=self.frame_index,
            next_reward=0.0,  # Placeholder reward
            next_done=False,
        )

    def log_to_rerun(self, robot_data: RobotData):
        """Log robot data to Rerun for visualization."""
        try:
            # Log joint values as time series with proper time tracking
            for i, (joint_name, obs_val, act_val) in enumerate(
                zip(self.joint_names, robot_data.observation_state, robot_data.action)
            ):
                rr.log(f"robot_data/joints/{joint_name}", rr.Scalars(obs_val))
                rr.log(f"robot_data/joints/{joint_name}_action", rr.Scalars(act_val))

        except Exception as e:
            self.logger.error(f"Failed to log to Rerun: {e}")
            # Don't raise the exception to prevent stopping the collection

    def collect_data(self):
        """Collect data continuously."""
        self.logger.info("Starting data collection")
        self.is_collecting = True
        self.frame_index = 0
        self.start_time = time.time()

        frame_interval = 1.0 / self.fps

        # Initialize joint names on first data collection
        if not self.joint_names:
            action_joints = self.get_joint_data(self.action_server_url)
            if action_joints:
                self.joint_names = list(action_joints.keys())
                self.logger.info(f"Initialized with {len(self.joint_names)} joints")

        while self.is_collecting and not should_stop:
            current_time = time.time() - self.start_time

            # Get joint data from both servers
            action_joints = self.get_joint_data(self.action_server_url)
            state_joints = self.get_joint_data(self.state_server_url)

            if not action_joints or not state_joints:
                self.logger.warning("Failed to get joint data, skipping frame")
                time.sleep(frame_interval)
                continue

            # Create robot data in the specified format
            robot_data = self.create_robot_data(
                action_joints, state_joints, current_time
            )

            # Log to Rerun
            self.log_to_rerun(robot_data)

            self.frame_index += 1
            time.sleep(frame_interval)

        self.logger.info(f"Data collection completed: {self.frame_index} frames")

    def cleanup(self):
        """Clean up resources before exit."""
        try:
            self.is_collecting = False
            self.logger.info("Cleanup completed")
        except Exception as e:
            self.logger.warning(f"Cleanup warning: {e}")

    def start_collection(self):
        """Start data collection."""
        self.logger.info("Starting data collection")
        self.collect_data()


def main():
    """Main function to run the robot rerun visualizer."""
    # Global variable to track if we should stop
    global should_stop
    should_stop = False

    parser = argparse.ArgumentParser(description="Visualize robot data in Rerun")
    parser.add_argument(
        "--action-server",
        default="http://localhost:5000",
        help="URL of the action server (default: http://localhost:5000)",
    )
    parser.add_argument(
        "--state-server",
        default="http://localhost:5001",
        help="URL of the state visualizer server (default: http://localhost:5001)",
    )
    parser.add_argument(
        "--rerun-endpoint",
        default="rerun+http://localhost:9876/proxy",
        help="Rerun gRPC endpoint URL (default: rerun+http://localhost:9876/proxy)",
    )
    parser.add_argument(
        "--fps",
        type=int,
        default=10)

    parser.add_argument(
        "--task-description",
        default="Robot Control Task",
        help="Description of the current task",
    )

    args = parser.parse_args()

    # Create visualizer
    visualizer = RobotRerunVisualizer(
        action_server_url=args.action_server,
        state_server_url=args.state_server,
        rerun_endpoint=args.rerun_endpoint,
        fps=args.fps,
        task_description=args.task_description,
    )

    try:
        # Start data collection
        visualizer.start_collection()
    finally:
        # Always cleanup
        visualizer.cleanup()
        print("👋 Goodbye!")


if __name__ == "__main__":
    main()
