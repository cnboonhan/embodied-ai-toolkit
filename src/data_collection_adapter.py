"""
Data Collection Adapter

Abstract class for collecting data from action and state servers.
Provides interface for getting actions, states, and managing datasets.
"""

from abc import ABC, abstractmethod
from typing import Any, Dict, List, Optional
import time
import threading
from dataclasses import dataclass


@dataclass
class DataPoint:
    """Represents a single data point with action and state."""
    timestamp: float
    action: Dict[str, Any]
    state: Dict[str, Any]
    metadata: Optional[Dict[str, Any]] = None


class DataCollectionAdapter(ABC):
    """
    Abstract base class for data collection from action and state servers.
    
    This class provides a framework for collecting synchronized data from
    action and state servers at a specified frequency.
    """
    
    def __init__(self, action_server_url: str, state_server_url: str, hz: float = 10.0):
        """
        Initialize the data collection adapter.
        
        Args:
            action_server_url: URL of the action server (e.g., "http://localhost:5000")
            state_server_url: URL of the state server (e.g., "http://localhost:5001")
            hz: Collection frequency in Hertz (default: 10.0)
        """
        self.action_server_url = action_server_url
        self.state_server_url = state_server_url
        self.hz = hz
        self.interval = 1.0 / hz
        
        # Data collection state
        self.is_collecting = False
        self.collection_thread: Optional[threading.Thread] = None
        
    @abstractmethod
    def get_action(self) -> Dict[str, Any]:
        """
        Get current action data from the action server.
        
        Returns:
            Dictionary containing action data (e.g., joint positions, velocities)
        """
        pass
    
    @abstractmethod
    def get_state(self) -> Dict[str, Any]:
        """
        Get current state data from the state server.
        
        Returns:
            Dictionary containing state data (e.g., joint positions, sensor readings)
        """
        pass
    
    @abstractmethod
    def create_dataset(self, name: str, description: str = "") -> str:
        """
        Create a new dataset for storing collected data.
        
        Args:
            name: Name of the dataset
            description: Optional description of the dataset
            
        Returns:
            Dataset identifier
        """
        pass
    
    @abstractmethod
    def add_to_dataset(self, dataset_id: str, data_point: DataPoint) -> bool:
        """
        Add a data point to the specified dataset.
        
        Args:
            dataset_id: Identifier of the dataset to add to
            data_point: DataPoint object containing action, state, and metadata
            
        Returns:
            True if successfully added, False otherwise
        """
        pass
    
    def start_collection(self) -> None:
        """
        Start continuous data collection at the specified frequency.
        """
        if self.is_collecting:
            print("⚠️  Data collection is already running")
            return
            
        self.is_collecting = True
        self.collection_thread = threading.Thread(target=self._collection_loop, daemon=True)
        self.collection_thread.start()
        print(f"🚀 Started data collection at {self.hz} Hz")
    
    def stop_collection(self) -> None:
        """
        Stop continuous data collection.
        """
        if not self.is_collecting:
            print("⚠️  Data collection is not running")
            return
            
        self.is_collecting = False
        if self.collection_thread:
            self.collection_thread.join(timeout=1.0)
        print("🛑 Stopped data collection")
    
    def _collection_loop(self) -> None:
        """
        Internal method that runs the continuous collection loop.
        """
        while self.is_collecting:
            try:
                # Get current action and state
                action = self.get_action()
                state = self.get_state()
                
                # Create data point
                data_point = DataPoint(
                    timestamp=time.time(),
                    action=action,
                    state=state
                )
                
                # Let subclasses handle dataset management
                self._handle_data_point(data_point)
                
                # Sleep for the specified interval
                time.sleep(self.interval)
                
            except Exception as e:
                print(f"❌ Error during data collection: {e}")
                time.sleep(self.interval)
    
    @abstractmethod
    def _handle_data_point(self, data_point: DataPoint) -> None:
        """
        Handle a collected data point. This method should be implemented by subclasses
        to determine how to process and store the data point.
        
        Args:
            data_point: The collected data point
        """
        pass
