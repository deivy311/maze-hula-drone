#!/usr/bin/env python3
"""
Python interface for Gazebo simulation using ROS or Gazebo API.
"""

import rospy
import cv2
from cv_bridge import CvBridge
from sensor_msgs.msg import Image, CameraInfo
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist, Pose
import numpy as np
from typing import Optional, Tuple, List
from markers import MarkerPlacement
from detect import MarkerDetector


class GazeboInterface:
    """Interface to Gazebo simulation via ROS."""
    
    def __init__(self, marker_placement: MarkerPlacement):
        """
        Initialize Gazebo interface.
        
        Args:
            marker_placement: Marker placement for decoding IDs
        """
        self.marker_placement = marker_placement
        self.bridge = CvBridge()
        self.detector = MarkerDetector()
        
        # Camera data
        self.camera_image: Optional[np.ndarray] = None
        self.camera_info: Optional[CameraInfo] = None
        
        # Drone state
        self.drone_pose: Optional[Pose] = None
        self.drone_odom: Optional[Odometry] = None
        
        # ROS setup
        rospy.init_node('maze_drone_controller', anonymous=True)
        
        # Subscribers
        self.image_sub = rospy.Subscriber(
            '/drone/camera/image_raw',
            Image,
            self._image_callback
        )
        self.camera_info_sub = rospy.Subscriber(
            '/drone/camera/camera_info',
            CameraInfo,
            self._camera_info_callback
        )
        self.odom_sub = rospy.Subscriber(
            '/drone/odom',
            Odometry,
            self._odom_callback
        )
        
        # Publishers
        self.cmd_vel_pub = rospy.Publisher(
            '/drone/cmd_vel',
            Twist,
            queue_size=10
        )
        
        # Wait for topics
        rospy.loginfo("Waiting for camera topics...")
        rospy.wait_for_message('/drone/camera/image_raw', Image, timeout=10)
        rospy.loginfo("Gazebo interface ready")
    
    def _image_callback(self, msg: Image) -> None:
        """Callback for camera image."""
        try:
            self.camera_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except Exception as e:
            rospy.logerr(f"Error converting image: {e}")
    
    def _camera_info_callback(self, msg: CameraInfo) -> None:
        """Callback for camera info."""
        self.camera_info = msg
    
    def _odom_callback(self, msg: Odometry) -> None:
        """Callback for odometry."""
        self.drone_odom = msg
        self.drone_pose = msg.pose.pose
    
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
        
        x = self.drone_pose.position.x
        y = self.drone_pose.position.y
        z = self.drone_pose.position.z
        
        # Extract yaw from quaternion
        q = self.drone_pose.orientation
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
    
    def shutdown(self) -> None:
        """Shutdown interface."""
        rospy.signal_shutdown("Simulation complete")


def main():
    """Test the interface."""
    from maze import Maze
    
    # Create a dummy marker placement for testing
    maze = Maze(5, 5)
    marker_placement = MarkerPlacement(maze)
    
    try:
        interface = GazeboInterface(marker_placement)
        
        rate = rospy.Rate(30)  # 30 Hz
        
        while not rospy.is_shutdown():
            # Get camera frame
            frame = interface.get_camera_frame()
            if frame is not None:
                # Detect markers
                marker_ids, marker_corners = interface.detect_markers()
                print(f"Detected {len(marker_ids)} markers: {marker_ids}")
                
                # Get position
                pos = interface.get_drone_position()
                if pos is not None:
                    print(f"Drone position: ({pos[0]:.2f}, {pos[1]:.2f}, yaw={pos[2]:.2f})")
            
            rate.sleep()
    
    except rospy.ROSInterruptException:
        pass
    except KeyboardInterrupt:
        interface.shutdown()


if __name__ == '__main__':
    main()
