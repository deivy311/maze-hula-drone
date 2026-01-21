"""
Synthetic camera simulation with perspective projection.
Markers are on the floor (z=0).
"""

import cv2
import numpy as np
from typing import Tuple, Optional, List
from markers import MarkerPlacement


class CameraSim:
    """Synthetic downward-facing camera simulation."""
    
    def __init__(
        self,
        width: int = 640,
        height: int = 480,
        fx: float = 500.0,
        fy: float = 500.0,
        cx: Optional[float] = None,
        cy: Optional[float] = None,
        noise_level: float = 0.0,
        blur_sigma: float = 0.0
    ):
        """
        Initialize camera simulation.
        
        Args:
            width: Image width in pixels
            height: Image height in pixels
            fx: Focal length X
            fy: Focal length Y
            cx: Principal point X (default: width/2)
            cy: Principal point Y (default: height/2)
            noise_level: Gaussian noise standard deviation (0 = no noise)
            blur_sigma: Gaussian blur sigma (0 = no blur)
        """
        self.width = width
        self.height = height
        self.fx = fx
        self.fy = fy
        self.cx = cx if cx is not None else width / 2.0
        self.cy = cy if cy is not None else height / 2.0
        self.noise_level = noise_level
        self.blur_sigma = blur_sigma
        
        # Camera intrinsics matrix
        self.K = np.array([
            [fx, 0, cx],
            [0, fy, cy],
            [0, 0, 1]
        ], dtype=np.float32)
    
    def generate_frame(
        self,
        x: float,
        y: float,
        z: float,
        yaw: float,
        marker_placement: MarkerPlacement,
        marker_size: float = 0.1
    ) -> np.ndarray:
        """
        Generate a camera frame from drone position.
        Markers are on the floor at z=0.
        
        Args:
            x: Drone X position in meters
            y: Drone Y position in meters
            z: Drone Z position (height) in meters
            yaw: Drone yaw angle in radians
            marker_placement: Marker placement manager
            marker_size: Size of markers in meters
        
        Returns:
            BGR image (uint8)
        """
        # Create blank white image
        img = np.ones((self.height, self.width, 3), dtype=np.uint8) * 255
        
        # Get visible markers (increase range for better detection)
        visible_markers = marker_placement.get_markers_in_range(x, y, z, max_range=5.0)
        
        # Render each visible marker
        for marker_id, mx, my, myaw, quadrant in visible_markers:
            marker_img = self._project_marker(
                x, y, z, yaw,
                mx, my, myaw,
                marker_size,
                marker_placement,
                marker_id
            )
            
            if marker_img is not None:
                # Blend marker into image
                self._blend_marker(img, marker_img)
        
        # Apply noise
        if self.noise_level > 0:
            noise = np.random.normal(0, self.noise_level, img.shape).astype(np.float32)
            img = np.clip(img.astype(np.float32) + noise, 0, 255).astype(np.uint8)
        
        # Apply blur
        if self.blur_sigma > 0:
            kernel_size = int(6 * self.blur_sigma) + 1
            if kernel_size % 2 == 0:
                kernel_size += 1
            img = cv2.GaussianBlur(img, (kernel_size, kernel_size), self.blur_sigma)
        
        return img
    
    def _project_marker(
        self,
        drone_x: float,
        drone_y: float,
        drone_z: float,
        drone_yaw: float,
        marker_x: float,
        marker_y: float,
        marker_yaw: float,
        marker_size: float,
        marker_placement: MarkerPlacement,
        marker_id: int
    ) -> Optional[np.ndarray]:
        """
        Project a marker into the camera frame using homography.
        Marker is on the floor at z=0.
        
        Returns:
            Projected marker image or None if not visible
        """
        # Marker is on floor at z=0
        marker_z = 0.0
        
        # Calculate relative position
        dx = marker_x - drone_x
        dy = marker_y - drone_y
        dz = marker_z - drone_z  # Negative because camera is above floor
        
        # Rotate to camera frame (account for drone yaw)
        cos_yaw = np.cos(-drone_yaw)  # Negative because we rotate world to camera
        sin_yaw = np.sin(-drone_yaw)
        
        # Camera frame: X right, Y down, Z forward (pointing down)
        cam_x = dx * cos_yaw - dy * sin_yaw
        cam_y = dx * sin_yaw + dy * cos_yaw
        cam_z = dz
        
        # Check if marker is below camera (positive cam_z means below)
        if cam_z >= 0:  # Marker is at or above camera (shouldn't happen for floor markers)
            return None
        
        # Generate marker image
        marker_img = marker_placement.get_marker_image(marker_id, size_pixels=200)
        marker_h, marker_w = marker_img.shape
        
        # Define marker corners in marker frame (centered at origin, on floor)
        half_size = marker_size / 2.0
        marker_corners_3d = np.array([
            [-half_size, -half_size, 0],  # Top-left
            [half_size, -half_size, 0],   # Top-right
            [half_size, half_size, 0],    # Bottom-right
            [-half_size, half_size, 0]    # Bottom-left
        ], dtype=np.float32)
        
        # Rotate marker corners by marker yaw
        cos_myaw = np.cos(marker_yaw)
        sin_myaw = np.sin(marker_yaw)
        R_marker = np.array([
            [cos_myaw, -sin_myaw, 0],
            [sin_myaw, cos_myaw, 0],
            [0, 0, 1]
        ])
        marker_corners_3d = (R_marker @ marker_corners_3d.T).T
        
        # Translate to world position
        marker_corners_3d[:, 0] += marker_x
        marker_corners_3d[:, 1] += marker_y
        marker_corners_3d[:, 2] = 0.0  # All on floor
        
        # Transform to camera frame and project
        image_corners = []
        for corner in marker_corners_3d:
            # World to camera
            wdx = corner[0] - drone_x
            wdy = corner[1] - drone_y
            wdz = 0.0 - drone_z  # Floor is at z=0
            
            cam_corner_x = wdx * cos_yaw - wdy * sin_yaw
            cam_corner_y = wdx * sin_yaw + wdy * cos_yaw
            cam_corner_z = wdz
            
            if cam_corner_z >= 0:
                return None  # Behind or at camera
            
            # Project to image plane
            u = self.fx * cam_corner_x / cam_corner_z + self.cx
            v = self.fy * cam_corner_y / cam_corner_z + self.cy
            image_corners.append([u, v])
        
        image_corners = np.array(image_corners, dtype=np.float32)
        
        # Check if marker is in view
        if np.any(image_corners[:, 0] < 0) or np.any(image_corners[:, 0] >= self.width):
            return None
        if np.any(image_corners[:, 1] < 0) or np.any(image_corners[:, 1] >= self.height):
            return None
        
        # Compute homography
        src_points = np.array([
            [0, 0],
            [marker_w, 0],
            [marker_w, marker_h],
            [0, marker_h]
        ], dtype=np.float32)
        
        dst_points = image_corners[:, :2]
        
        # Compute homography matrix
        H, _ = cv2.findHomography(src_points, dst_points)
        
        if H is None:
            return None
        
        # Warp marker image
        warped = cv2.warpPerspective(
            marker_img,
            H,
            (self.width, self.height),
            flags=cv2.INTER_LINEAR,
            borderMode=cv2.BORDER_TRANSPARENT
        )
        
        return warped
    
    def _blend_marker(self, img: np.ndarray, marker: np.ndarray) -> None:
        """
        Blend marker into image (simple overwrite for now).
        
        Args:
            img: Target BGR image (modified in place)
            marker: Grayscale marker image to blend
        """
        # Convert marker to 3-channel if needed
        if len(marker.shape) == 2:
            marker_3ch = cv2.cvtColor(marker, cv2.COLOR_GRAY2BGR)
        else:
            marker_3ch = marker
        
        # Simple overwrite (marker is black on white)
        mask = marker < 128  # Black pixels
        if len(mask.shape) == 2:
            mask = np.stack([mask, mask, mask], axis=2)
        
        img[mask] = marker_3ch[mask]
