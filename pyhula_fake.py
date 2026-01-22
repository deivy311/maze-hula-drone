#!/usr/bin/env python3
"""
Fake pyhula library for Gazebo simulation.
This module provides a drop-in replacement for pyhula.UserApi() that works with Gazebo.
"""

import time
import math
import threading
from typing import Optional, Tuple, Dict
import numpy as np

try:
    import rclpy
    from rclpy.node import Node
    from sensor_msgs.msg import Image
    from nav_msgs.msg import Odometry
    from geometry_msgs.msg import Twist
    from cv_bridge import CvBridge
    ROS2_AVAILABLE = True
except ImportError:
    try:
        import rospy
        from sensor_msgs.msg import Image
        from nav_msgs.msg import Odometry
        from geometry_msgs.msg import Twist
        from cv_bridge import CvBridge
        ROS2_AVAILABLE = False
        ROS1_AVAILABLE = True
    except ImportError:
        ROS2_AVAILABLE = False
        ROS1_AVAILABLE = False
        print("Warning: ROS not available. pyhula_fake will not work properly.")


class UserApi:
    """
    Fake UserApi class that mimics pyhula.UserApi() for Gazebo simulation.
    
    This class provides the same interface as the real pyhula library but
    uses Gazebo/ROS for actual control and sensing.
    """
    
    def __init__(self):
        """Initialize the fake API."""
        self.connected = False
        self.taken_off = False
        
        # Current position (in meters, will be converted to cm for API)
        self.x = 0.0
        self.y = 0.0
        self.z = 1.0  # Default altitude
        self.yaw = 0.0  # in radians
        
        # Movement state
        self.moving = False
        self.move_start_time = 0.0
        self.move_duration = 0.0
        self.move_vx = 0.0
        self.move_vy = 0.0
        self.move_omega = 0.0
        
        # ROS interface
        self.ros_node = None
        self.cmd_vel_pub = None
        self.odom_sub = None
        self.drone_odom = None
        self.spin_thread = None
        
        # Barrier/obstacle detection (simplified - always returns no obstacles)
        self.barrier_enabled = False
        
        # Plane ID for video (fake)
        self.plane_id = 1
        
        # QR mode
        self.qr_mode = 0
        
        # RTP mode (for video)
        self.rtp_mode = 0
        
        # Cell size for movement calculations (default 0.6m = 60cm)
        self.cell_size_m = 0.6
        
    def connect(self) -> bool:
        """
        Connect to the drone (simulated).
        
        Returns:
            True if connection successful
        """
        if self.connected:
            return True
        
        # Initialize ROS if available
        if ROS2_AVAILABLE:
            try:
                if not rclpy.ok():
                    rclpy.init()
                self.ros_node = Node('pyhula_fake_node')
                self.cmd_vel_pub = self.ros_node.create_publisher(Twist, '/cmd_vel', 10)
                self.odom_sub = self.ros_node.create_subscription(
                    Odometry, '/odom', self._odom_callback, 10
                )
                # Start spinning in a separate thread
                self.spin_thread = threading.Thread(target=rclpy.spin, args=(self.ros_node,))
                self.spin_thread.daemon = True
                self.spin_thread.start()
                # Wait a bit for topics to be ready
                time.sleep(0.5)
            except Exception as e:
                print(f"[PYHULA_FAKE] Warning: Failed to initialize ROS 2: {e}")
                print("[PYHULA_FAKE] Make sure Gazebo is running and ROS bridge is configured")
        elif ROS1_AVAILABLE:
            try:
                if not rospy.get_node_uri():
                    rospy.init_node('pyhula_fake_node', anonymous=True)
                self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
                self.odom_sub = rospy.Subscriber('/odom', Odometry, self._odom_callback)
                rospy.sleep(0.5)  # Wait for connection
            except Exception as e:
                print(f"[PYHULA_FAKE] Warning: Failed to initialize ROS 1: {e}")
                print("[PYHULA_FAKE] Make sure Gazebo is running and ROS bridge is configured")
        else:
            print("[PYHULA_FAKE] Warning: ROS not available. Simulation will not work properly.")
            print("[PYHULA_FAKE] Please install ROS 2 (Jazzy) or ROS 1 and configure the bridge.")
        
        self.connected = True
        print("[PYHULA_FAKE] Connected to Gazebo simulation")
        print("[PYHULA_FAKE] Note: If you don't see Gazebo, make sure:")
        print("[PYHULA_FAKE]   1. Gazebo is running: gz sim gazebo/worlds/generated_maze.world")
        print("[PYHULA_FAKE]   2. ROS bridge is configured: source setup_ros2_bridge.sh")
        return True
    
    def _odom_callback(self, msg):
        """Callback for odometry updates."""
        if ROS2_AVAILABLE:
            x = msg.pose.pose.position.x
            y = msg.pose.pose.position.y
            z = msg.pose.pose.position.z
            
            # Extract yaw from quaternion
            q = msg.pose.pose.orientation
            yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
        else:  # ROS1
            x = msg.pose.pose.position.x
            y = msg.pose.pose.position.y
            z = msg.pose.pose.position.z
            
            q = msg.pose.pose.orientation
            yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
        
        self.x = x
        self.y = y
        self.z = z
        self.yaw = yaw
        self.drone_odom = msg
    
    def Plane_cmd_switch_QR(self, mode: int) -> None:
        """
        Switch QR code detection mode.
        
        Args:
            mode: 0 = disable, 1 = enable
        """
        self.qr_mode = mode
        print(f"[PYHULA_FAKE] QR mode set to {mode}")
    
    def single_fly_barrier_aircraft(self, enable: bool) -> None:
        """
        Enable/disable barrier (obstacle) detection.
        
        Args:
            enable: True to enable, False to disable
        """
        self.barrier_enabled = enable
        print(f"[PYHULA_FAKE] Barrier detection {'enabled' if enable else 'disabled'}")
    
    def Plane_getBarrier(self) -> Dict[str, bool]:
        """
        Get barrier/obstacle detection status.
        
        Returns:
            Dictionary with keys: "forward", "right", "left", "back"
            Values are True if obstacle detected, False otherwise
        """
        # In simulation, we don't have real obstacle detection yet
        # Return all False (no obstacles) for now
        return {
            "forward": False,
            "right": False,
            "left": False,
            "back": False
        }
    
    def get_coordinate(self) -> Tuple[float, float, float]:
        """
        Get current drone position.
        
        Returns:
            Tuple (x_cm, y_cm, z_cm) in centimeters
        """
        # Convert from meters to centimeters
        x_cm = self.x * 100.0
        y_cm = self.y * 100.0
        z_cm = self.z * 100.0
        return (x_cm, y_cm, z_cm)
    
    def get_yaw(self) -> Tuple[float, ...]:
        """
        Get current yaw angle.
        
        Returns:
            Tuple (yaw_deg, ...) where yaw_deg is in degrees
        """
        yaw_deg = math.degrees(self.yaw)
        return (yaw_deg,)
    
    def single_fly_takeoff(self) -> None:
        """Command drone to take off."""
        if not self.connected:
            print("[PYHULA_FAKE] Warning: Not connected. Call connect() first.")
            return
        
        self.taken_off = True
        print("[PYHULA_FAKE] Takeoff command sent")
        # In simulation, the drone should already be at altitude
        # We can optionally send a small upward velocity command
        self._send_velocity(0.0, 0.0, 0.5)  # Small upward velocity
        time.sleep(0.5)
        self._send_velocity(0.0, 0.0, 0.0)  # Stop
    
    def single_fly_touchdown(self) -> None:
        """Command drone to land."""
        if not self.connected:
            print("[PYHULA_FAKE] Warning: Not connected.")
            return
        
        print("[PYHULA_FAKE] Landing command sent")
        # Send downward velocity
        self._send_velocity(0.0, 0.0, -0.3)
        time.sleep(1.0)
        self._send_velocity(0.0, 0.0, 0.0)  # Stop
        self.taken_off = False
    
    def single_fly_forward(self, step: float, speed: float = 60.0) -> None:
        """
        Move forward.
        
        Args:
            step: Distance in centimeters
            speed: Speed (not used in simulation, but kept for compatibility)
        """
        self._move_direction("forward", step, speed)
    
    def single_fly_right(self, step: float, speed: float = 60.0) -> None:
        """
        Move right.
        
        Args:
            step: Distance in centimeters
            speed: Speed (not used in simulation, but kept for compatibility)
        """
        self._move_direction("right", step, speed)
    
    def single_fly_left(self, step: float, speed: float = 60.0) -> None:
        """
        Move left.
        
        Args:
            step: Distance in centimeters
            speed: Speed (not used in simulation, but kept for compatibility)
        """
        self._move_direction("left", step, speed)
    
    def single_fly_back(self, step: float, speed: float = 60.0) -> None:
        """
        Move backward.
        
        Args:
            step: Distance in centimeters
            speed: Speed (not used in simulation, but kept for compatibility)
        """
        self._move_direction("back", step, speed)
    
    def _move_direction(self, direction: str, step_cm: float, speed: float) -> None:
        """
        Internal method to move in a specific direction.
        
        Args:
            direction: "forward", "right", "left", "back"
            step_cm: Distance in centimeters
            speed: Speed parameter (for compatibility, used as velocity magnitude)
        """
        if not self.connected:
            print("[PYHULA_FAKE] Warning: Not connected.")
            return
        
        # Convert step from cm to meters
        step_m = step_cm / 100.0
        
        # Use speed parameter if provided (convert from cm/s to m/s, default 60cm/s = 0.6 m/s)
        if speed > 0:
            vel_mag = speed / 100.0  # Convert cm/s to m/s
        else:
            vel_mag = 0.6  # Default 0.6 m/s
        
        # Calculate velocity components based on current yaw
        # In the drone's body frame:
        # forward = +x (in body frame, along heading)
        # right = +y (in body frame, perpendicular right)
        # back = -x
        # left = -y
        
        # Convert to world frame
        cos_yaw = math.cos(self.yaw)
        sin_yaw = math.sin(self.yaw)
        
        if direction == "forward":
            vx = cos_yaw * vel_mag
            vy = sin_yaw * vel_mag
        elif direction == "right":
            vx = -sin_yaw * vel_mag
            vy = cos_yaw * vel_mag
        elif direction == "left":
            vx = sin_yaw * vel_mag
            vy = -cos_yaw * vel_mag
        elif direction == "back":
            vx = -cos_yaw * vel_mag
            vy = -sin_yaw * vel_mag
        else:
            print(f"[PYHULA_FAKE] Unknown direction: {direction}")
            return
        
        # Calculate duration based on step size and velocity
        duration = abs(step_m) / vel_mag if vel_mag > 0 else 0.1
        
        # Send velocity command continuously for the duration
        start_time = time.time()
        rate = 10  # Hz - send commands at 10Hz
        dt = 1.0 / rate
        
        while (time.time() - start_time) < duration:
            self._send_velocity(vx, vy, 0.0)
            time.sleep(dt)
        
        # Stop
        self._send_velocity(0.0, 0.0, 0.0)
        
        print(f"[PYHULA_FAKE] Moved {direction} by {step_cm}cm at {speed}cm/s (duration: {duration:.2f}s)")
    
    def single_fly_turnleft(self, angle_deg: float) -> None:
        """
        Rotate left (counter-clockwise).
        
        Args:
            angle_deg: Angle in degrees
        """
        self._rotate("left", angle_deg)
    
    def single_fly_turnright(self, angle_deg: float) -> None:
        """
        Rotate right (clockwise).
        
        Args:
            angle_deg: Angle in degrees
        """
        self._rotate("right", angle_deg)
    
    def _rotate(self, direction: str, angle_deg: float) -> None:
        """
        Internal method to rotate the drone.
        
        Args:
            direction: "left" or "right"
            angle_deg: Angle in degrees
        """
        if not self.connected:
            print("[PYHULA_FAKE] Warning: Not connected.")
            return
        
        # Convert to radians
        angle_rad = math.radians(angle_deg)
        
        # Angular velocity (rad/s)
        omega = 0.5  # 0.5 rad/s = ~28.6 deg/s
        
        # Determine rotation direction
        if direction == "left":
            omega = -omega  # Counter-clockwise (positive yaw)
        elif direction == "right":
            omega = omega  # Clockwise (negative yaw)
        else:
            print(f"[PYHULA_FAKE] Unknown rotation direction: {direction}")
            return
        
        # Calculate duration
        duration = abs(angle_rad) / abs(omega)
        
        # Send angular velocity command
        twist = Twist()
        twist.linear.x = 0.0
        twist.linear.y = 0.0
        twist.linear.z = 0.0
        twist.angular.x = 0.0
        twist.angular.y = 0.0
        twist.angular.z = omega
        
        if self.cmd_vel_pub is not None:
            if ROS2_AVAILABLE:
                self.cmd_vel_pub.publish(twist)
            else:  # ROS1
                self.cmd_vel_pub.publish(twist)
        
        # Wait for rotation to complete
        time.sleep(duration)
        
        # Stop
        twist.angular.z = 0.0
        if self.cmd_vel_pub is not None:
            if ROS2_AVAILABLE:
                self.cmd_vel_pub.publish(twist)
            else:  # ROS1
                self.cmd_vel_pub.publish(twist)
        
        print(f"[PYHULA_FAKE] Rotated {direction} by {angle_deg}°")
    
    def _send_velocity(self, vx: float, vy: float, vz: float) -> None:
        """
        Send velocity command to Gazebo.
        
        Args:
            vx: Velocity in x direction (m/s)
            vy: Velocity in y direction (m/s)
            vz: Velocity in z direction (m/s)
        """
        if self.cmd_vel_pub is None:
            return
        
        try:
            twist = Twist()
            twist.linear.x = vx
            twist.linear.y = vy
            twist.linear.z = vz
            twist.angular.x = 0.0
            twist.angular.y = 0.0
            twist.angular.z = 0.0
            
            if ROS2_AVAILABLE:
                if rclpy.ok() and self.cmd_vel_pub is not None:
                    self.cmd_vel_pub.publish(twist)
            else:  # ROS1
                if not rospy.is_shutdown() and self.cmd_vel_pub is not None:
                    self.cmd_vel_pub.publish(twist)
        except Exception as e:
            # Silently ignore errors during shutdown
            pass
    
    def get_plane_id(self) -> int:
        """
        Get plane/drone ID (for video streaming).
        
        Returns:
            Plane ID (fake, returns 1)
        """
        return self.plane_id
    
    def Plane_cmd_swith_rtp(self, mode: int) -> None:
        """
        Switch RTP video streaming mode.
        
        Args:
            mode: 0 = disable, 1 = enable
        """
        self.rtp_mode = mode
        print(f"[PYHULA_FAKE] RTP mode set to {mode}")
    
    def UAV_GetCoordinate(self) -> Tuple[float, float, float]:
        """Alias for get_coordinate() for compatibility."""
        return self.get_coordinate()
    
    def UAV_GetYaw(self) -> Tuple[float, ...]:
        """Alias for get_yaw() for compatibility."""
        return self.get_yaw()
    
    def UAV_GetFlyState(self) -> int:
        """
        Get current flight state.
        
        Returns:
            0 = landed, 1 = flying
        """
        return 1 if self.taken_off else 0
    
    def shutdown(self) -> None:
        """Shutdown ROS node."""
        try:
            if ROS2_AVAILABLE and self.ros_node is not None:
                if rclpy.ok():
                    self.ros_node.destroy_node()
                    rclpy.shutdown()
        except Exception:
            pass
        try:
            if ROS1_AVAILABLE:
                if not rospy.is_shutdown():
                    rospy.signal_shutdown("pyhula_fake shutdown")
        except Exception:
            pass
