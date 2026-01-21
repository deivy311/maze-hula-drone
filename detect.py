"""
ArUco marker detection wrapper.
"""

import cv2
import numpy as np
from typing import List, Tuple, Optional
from markers import MarkerPlacement


class MarkerDetector:
    """Wrapper for ArUco marker detection."""
    
    def __init__(self, dictionary_id: int = cv2.aruco.DICT_4X4_1000):
        """
        Initialize detector.
        
        Args:
            dictionary_id: ArUco dictionary ID (must match marker placement)
        """
        self.dictionary = cv2.aruco.getPredefinedDictionary(dictionary_id)
        self.detector_params = cv2.aruco.DetectorParameters()
        self.detector = cv2.aruco.ArucoDetector(self.dictionary, self.detector_params)
    
    def detect(self, image: np.ndarray) -> Tuple[List[int], List[np.ndarray], Optional[np.ndarray]]:
        """
        Detect ArUco markers in image.
        
        Args:
            image: Grayscale or BGR image
        
        Returns:
            Tuple of (marker_ids, marker_corners, rejected_candidates)
        """
        # Convert to grayscale if needed
        if len(image.shape) == 3:
            gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        else:
            gray = image
        
        # Detect markers
        corners, ids, rejected = self.detector.detectMarkers(gray)
        
        if ids is None or len(ids) == 0:
            return ([], [], rejected)
        
        # Convert to lists
        marker_ids = [int(id[0]) for id in ids]
        marker_corners = [corner[0] for corner in corners]
        
        return (marker_ids, marker_corners, rejected)
    
    def decode_detections(
        self,
        marker_ids: List[int],
        marker_placement: MarkerPlacement
    ) -> List[Tuple[int, int, int]]:
        """
        Decode detected marker IDs to cell positions and quadrants.
        
        Args:
            marker_ids: List of detected marker IDs
            marker_placement: Marker placement manager
        
        Returns:
            List of (row, col, quadrant) tuples
        """
        decoded = []
        for marker_id in marker_ids:
            result = marker_placement.decode_id(marker_id)
            if result is not None:
                decoded.append(result)
        return decoded
    
    def estimate_pose(
        self,
        marker_id: int,
        marker_corners: np.ndarray,
        camera_matrix: np.ndarray,
        dist_coeffs: Optional[np.ndarray] = None,
        marker_size: float = 0.1
    ) -> Optional[Tuple[np.ndarray, np.ndarray]]:
        """
        Estimate pose of a detected marker.
        
        Args:
            marker_id: Marker ID
            marker_corners: 4x2 array of corner pixel coordinates
            camera_matrix: 3x3 camera intrinsics matrix
            dist_coeffs: Distortion coefficients (optional)
            marker_size: Size of marker in meters
        
        Returns:
            Tuple (rvec, tvec) or None if estimation fails
        """
        if dist_coeffs is None:
            dist_coeffs = np.zeros((4, 1))
        
        # Define marker object points (in marker frame)
        obj_points = np.array([
            [-marker_size/2, marker_size/2, 0],
            [marker_size/2, marker_size/2, 0],
            [marker_size/2, -marker_size/2, 0],
            [-marker_size/2, -marker_size/2, 0]
        ], dtype=np.float32)
        
        # Estimate pose
        success, rvec, tvec = cv2.solvePnP(
            obj_points,
            marker_corners,
            camera_matrix,
            dist_coeffs
        )
        
        if success:
            return (rvec, tvec)
        
        return None
