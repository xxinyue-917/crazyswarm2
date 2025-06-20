import threading
import time
from typing import Dict, Tuple, Optional, List
import rclpy
from rclpy.node import Node
from tf2_msgs.msg import TFMessage
import math


class PositionTracker(Node):
    """
    Lightweight ROS2 node for tracking drone positions and detecting if drones are flipped over.
    
    This class runs in a background thread and maintains a dictionary of
    current positions and orientations for all tracked drone frames.
    """
    
    def __init__(self, node_name: str = 'position_tracker'):
        """
        Initialize the position tracker.
        
        Args:
            node_name: Name for the ROS2 node
        """
        super().__init__(node_name)
        
        # Dictionary to store drone data: {frame_id: (x, y, z, qx, qy, qz, qw, timestamp, is_flipped)}
        self.drone_data: Dict[str, Tuple[float, float, float, float, float, float, float, float, bool]] = {}
        self._lock = threading.Lock()  # Thread-safe access to drone data
        
        # Create TF subscription
        self.tf_subscriber = self.create_subscription(
            TFMessage,
            '/tf',
            self.tf_callback,
            10
        )
        
        # Background spinning
        self.running = True
        self.spin_thread = threading.Thread(target=self._spin_worker, daemon=True)
        self.spin_thread.start()
        
        self.get_logger().info(f"Drone position tracker '{node_name}' started")
    
    def tf_callback(self, msg: TFMessage):
        """
        Callback for TF messages. Updates drone data dictionary.
        
        Args:
            msg: TF message containing transforms
        """
        current_time = time.time()
        
        with self._lock:
            for transform in msg.transforms:
                frame_id = transform.child_frame_id
                translation = transform.transform.translation
                rotation = transform.transform.rotation
                
                # Check if drone is flipped over based on orientation
                is_flipped = self._is_drone_flipped(rotation.x, rotation.y, rotation.z, rotation.w)
                
                # Store drone data with timestamp and flip status
                self.drone_data[frame_id] = (
                    translation.x,
                    translation.y, 
                    translation.z,
                    rotation.x,
                    rotation.y,
                    rotation.z,
                    rotation.w,
                    current_time,
                    is_flipped
                )
    
    def _is_drone_flipped(self, qx: float, qy: float, qz: float, qw: float) -> bool:
        """
        Determine if a drone is flipped over based on its quaternion orientation.
        
        A drone is considered flipped if its z-axis (up vector) is pointing downward.
        
        Args:
            qx, qy, qz, qw: Quaternion components
            
        Returns:
            True if drone is flipped over, False otherwise
        """
        # Convert quaternion to rotation matrix to get the z-axis direction
        # The z-axis of the drone in world coordinates is:
        # z_world = [2*(qx*qz + qw*qy), 2*(qy*qz - qw*qx), 1 - 2*(qx*qx + qy*qy)]
        
        z_component = 1 - 2 * (qx * qx + qy * qy)
        
        # If z-component is negative, the drone's up vector is pointing down (flipped)
        return z_component < -0.5  # Use threshold to account for slight tilts
    
    def _spin_worker(self):
        """Background worker thread for spinning the node."""
        while self.running and rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.01)
            time.sleep(0.001)
    
    def get_drone_position(self, drone_id: str) -> Optional[List[float]]:
        """
        Get the current position of a specific drone.
        
        Args:
            drone_id: The drone frame ID to get position for
            
        Returns:
            [x, y, z] list if drone exists, data is recent (within 2 seconds), 
            AND drone is not flipped over. Returns None otherwise.
        """
        with self._lock:
            if drone_id in self.drone_data:
                x, y, z, qx, qy, qz, qw, timestamp, is_flipped = self.drone_data[drone_id]
                
                # Only return data if it's recent (within 2 seconds) AND not flipped over
                if time.time() - timestamp < 2.0 and not is_flipped:
                    return [x, y, z]
            return None
    
    def get_tracked_drones(self) -> List[str]:
        """
        Get list of all tracked drone IDs.
        
        Returns:
            List of drone frame IDs currently being tracked
        """
        with self._lock:
            return list(self.drone_data.keys())
    
    def shutdown(self):
        """Shutdown the position tracker cleanly."""
        self.running = False
        if self.spin_thread.is_alive():
            self.spin_thread.join(timeout=1.0)
        self.get_logger().info("Drone position tracker shutdown")