"""
Joint State Adapter

This module provides a minimal abstract interface for joint state adapters.
The abstract class defines only the input/output contract, leaving all 
implementation details to concrete classes.
"""

from abc import ABC, abstractmethod
from typing import Dict, List, Any, Optional, Deque
from dataclasses import dataclass, field
from collections import deque
import copy


@dataclass
class JointData:
    """
    Container for individual joint information following the joint_state_schema.json structure.
    
    Based on the schema, this class contains exactly the fields defined:
    - value: Current position/value of the joint (required, type: number)
    - limits: Joint limits as [min, max] (required, type: array with exactly 2 numbers)
    
    The schema specifies "additionalProperties": false, so no other fields are allowed.
    """
    value: float
    limits: List[float]  # [min, max] - exactly 2 values required
    
    def __post_init__(self):
        """Validate joint data according to the schema constraints."""
        if len(self.limits) != 2:
            raise ValueError("Joint limits must contain exactly 2 values [min, max] as per schema")
        if self.limits[0] > self.limits[1]:
            raise ValueError("Joint limit minimum must be less than or equal to maximum")


@dataclass
class JointStateData:
    """Container for joint state information following the get_joints API structure."""
    joints: Dict[str, JointData]  
    timestamp: float


@dataclass 
class AdapterConfig:
    """Configuration parameters for the joint state adapter."""
    server_url: str = "http://127.0.0.1:5000/get_joints"
    exponential_alpha: float = 0.3
    history_length: int = 10
    change_tolerance: float = 0.005
    messaging_config: Dict[str, Any] = field(default_factory=dict)
    joint_name_mapping: str = ""


class JointStateAdapter(ABC):
    """
    Abstract base class for joint state adapters.
    
    This class defines only the input/output interface for adapting
    joint states between an HTTP API server and a messaging system.
    All implementation details are handled by concrete classes.
    """
    
    def __init__(self, config: AdapterConfig):
        self.config = config
        self.server_joint_state_history: Deque[JointStateData] = deque(maxlen=config.history_length)
    
    def add_to_server_joint_state_history(self, joint_state: JointStateData) -> None:
        self.server_joint_state_history.append(copy.deepcopy(joint_state))
    
    def get_history(self) -> List[JointStateData]:
        return list(self.server_joint_state_history)
    
    def clear_history(self) -> None:
        self.server_joint_state_history.clear()
    
    def apply_joint_reversal(self, joint_name: str, joint_data: JointData) -> float:
        """
        Apply joint reversal to a joint value based on its limits.
        
        This method reverses a joint value around the midpoint of its limits.
        For example, if limits are [-1, 1] and value is 0.5, the reversed value would be -0.5.
        
        Args:
            joint_name: Name of the joint (for logging purposes)
            joint_data: Joint data containing value and limits
            
        Returns:
            Reversed joint value
            
        Raises:
            ValueError: If joint limits are invalid
        """
        if len(joint_data.limits) != 2:
            raise ValueError(f"Joint {joint_name} must have exactly 2 limit values [min, max]")
        
        lower_limit, upper_limit = joint_data.limits
        midpoint = (lower_limit + upper_limit) / 2
        distance_from_midpoint = joint_data.value - midpoint
        reversed_value = midpoint - distance_from_midpoint
        
        return reversed_value
    
    def apply_joint_reversal_with_mapping(self, joint_mappings: Dict[str, Dict[str, Any]], 
                                        joint_state: JointStateData) -> JointStateData:
        """
        Apply joint reversal to multiple joints based on mapping configuration.
        
        Args:
            joint_mappings: Dictionary mapping API joint names to configuration including 'reverse' flag
            joint_state: Joint state data to process
            
        Returns:
            New JointStateData with reversed joints applied
        """
        # Create a deep copy to avoid modifying the original
        processed_state = copy.deepcopy(joint_state)
        
        for api_joint_name, mapping_info in joint_mappings.items():
            if api_joint_name in processed_state.joints and mapping_info.get('reverse', False):
                joint_data = processed_state.joints[api_joint_name]
                original_value = joint_data.value
                
                try:
                    reversed_value = self.apply_joint_reversal(api_joint_name, joint_data)
                    joint_data.value = reversed_value
                except ValueError as e:
                    # Log warning but continue with original value
                    # Concrete implementations can override this behavior
                    continue
        
        return processed_state
    
    def apply_exponential_smoothing(self, new_joint_state: JointStateData) -> JointStateData:
        """
        Apply exponential smoothing to joint state data using the full history.
        
        This method uses the full history to compute a weighted average where more recent
        values have higher weights according to the exponential_alpha parameter.
        
        Args:
            new_joint_state: The newest joint state data to smooth
            
        Returns:
            Smoothed joint state data
        """
        if not self.server_joint_state_history:
            # If no history, return the new data as-is
            return copy.deepcopy(new_joint_state)
        
        # Create a copy for the smoothed result
        smoothed_state = copy.deepcopy(new_joint_state)
        alpha = self.config.exponential_alpha
        
        # Apply exponential smoothing for each joint
        for joint_name, joint_data in smoothed_state.joints.items():
            # Collect historical values for this joint
            historical_values = []
            for hist_data in self.server_joint_state_history:
                if joint_name in hist_data.joints:
                    historical_values.append(hist_data.joints[joint_name].value)
            
            if historical_values:
                # Apply exponential smoothing using the full history
                # Start with the oldest value
                smoothed_value = historical_values[0]
                
                # Apply exponential smoothing iteratively through the history
                for i in range(1, len(historical_values)):
                    smoothed_value = alpha * historical_values[i] + (1 - alpha) * smoothed_value
                
                # Apply smoothing with the new value
                final_smoothed_value = alpha * joint_data.value + (1 - alpha) * smoothed_value
                
                # Update the smoothed state
                smoothed_state.joints[joint_name].value = final_smoothed_value
        
        return smoothed_state
    
    @abstractmethod
    def get_joint_state_from_server(self) -> Optional[JointStateData]:
        """
        Get current joint state data from the server.r
        
        Returns:
            Current joint state data from server or None if unavailable
        """
        pass
    
    @abstractmethod
    def get_joint_state_from_robot(self) -> Optional[JointStateData]:
        """
        Get current joint state data from the robot.
        
        Returns:
            Current joint state data from robot or None if unavailable
        """
        pass
    
    @abstractmethod
    def sync_joint_state_to_server(self, joint_state: JointStateData) -> None:
        """
        Sync/send joint state data to the server.
        
        Args:
            joint_state: Joint state data to sync to server
        """
        pass
    
    @abstractmethod
    def sync_joint_state_to_robot(self, joint_state: JointStateData) -> None:
        """
        Send motion commands to control the robot to match the joint state.
        
        Args:
            joint_state: Target joint state for the robot to achieve
        """
        pass
    
    @abstractmethod
    def start(self) -> None:
        """Start the adapter (begin spinning, listening, etc.)."""
        pass
    
    @abstractmethod
    def stop(self) -> None:
        """Stop the adapter and clean up resources."""
        pass
