#!/usr/bin/env python3
"""
Python interface for Gazebo Jazzy simulation using ROS 2.
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge
import cv2
import numpy as np
from typing import Optional, Tuple, List
from markers import MarkerPlacement
from detect import MarkerDetector


class GazeboInterfaceROS2(Node):
    """Interface to Gazebo Jazzy simulation via ROS 2."""
    
    def __init__(self, marker_placement: MarkerPlacement):
        """
        Initialize Gazebo interface.
        
        Args:
            marker_placement: Marker placement for decoding IDs
        """
        super().__init__('maze_drone_controller')
        self.marker_placement = marker_placement
        self.bridge = CvBridge()
        self.detector = MarkerDetector()
        
        # Camera data
        self.camera_image: Optional[np.ndarray] = None
        
        # Drone state
        self.drone_pose: Optional[Odometry] = None
        
        # Subscribers
        self.image_sub = self.create_subscription(
            Image,
            '/camera/image_raw',
            self._image_callback,
            10
        )
        self.odom_sub = self.create_subscription(
            Odometry,
            '/odom',
            self._odom_callback,
            10
        )
        
        # Publishers
        self.cmd_vel_pub = self.create_publisher(
            Twist,
            '/cmd_vel',
            10
        )
        
        self.get_logger().info("Gazebo ROS 2 interface ready")
    
    def _image_callback(self, msg: Image) -> None:
        """Callback for camera image."""
        try:
            self.camera_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except Exception as e:
            self.get_logger().error(f"Error converting image: {e}")
    
    def _odom_callback(self, msg: Odometry) -> None:
        """Callback for odometry."""
        self.drone_pose = msg
    
    def get_camera_frame(self) -> Optional[np.ndarray]:
        """
        Get current camera frame.
        
        Returns:
            BGR image or None
        """
        return self.camera_image
    
    def detect_markers(self) -> Tuple[List[int], List[np.ndarray]]:
        """
        Detect ArUco markers in current camera frame.
        
        Returns:
            Tuple of (marker_ids, marker_corners)
        """
        if self.camera_image is None:
            return ([], [])
        
        marker_ids, marker_corners, _ = self.detector.detect(self.camera_image)
        return (marker_ids, marker_corners)
    
    def get_drone_position(self) -> Optional[Tuple[float, float, float]]:
        """
        Get drone position.
        
        Returns:
            Tuple (x, y, yaw) or None
        """
        if self.drone_pose is None:
            return None
        
        x = self.drone_pose.pose.pose.position.x
        y = self.drone_pose.pose.pose.position.y
        
        # Extract yaw from quaternion
        q = self.drone_pose.pose.pose.orientation
        yaw = np.arctan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
        
        return (x, y, yaw)
    
    def set_velocity(self, v: float, omega: float) -> None:
        """
        Set drone velocity commands.
        
        Args:
            v: Forward velocity (m/s)
            omega: Angular velocity (rad/s)
        """
        twist = Twist()
        twist.linear.x = v
        twist.angular.z = omega
        self.cmd_vel_pub.publish(twist)


def main():
    """Test the interface."""
    from maze import Maze
    
    # Create a dummy marker placement for testing
    maze = Maze(5, 5)
    marker_placement = MarkerPlacement(maze)
    
    rclpy.init()
    
    try:
        interface = GazeboInterfaceROS2(marker_placement)
        
        # Spin in a separate thread
        import threading
        spin_thread = threading.Thread(target=rclpy.spin, args=(interface,))
        spin_thread.daemon = True
        spin_thread.start()
        
        import time
        while rclpy.ok():
            # Get camera frame
            frame = interface.get_camera_frame()
            if frame is not None:
                # Detect markers
                marker_ids, marker_corners = interface.detect_markers()
                interface.get_logger().info(f"Detected {len(marker_ids)} markers: {marker_ids}")
                
                # Get position
                pos = interface.get_drone_position()
                if pos is not None:
                    interface.get_logger().info(f"Drone position: ({pos[0]:.2f}, {pos[1]:.2f}, yaw={pos[2]:.2f})")
            
            time.sleep(0.1)
    
    except KeyboardInterrupt:
        pass
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()
