"""
ArUco marker placement and ID encoding module.
Markers are placed on the floor in a 2x2 pattern per cell.
"""

import math
import cv2
import numpy as np
from typing import List, Tuple, Dict, Optional
from maze import Maze


class MarkerPlacement:
    """Manages ArUco marker placement in the maze."""
    
    def __init__(self, maze: Maze, marker_size: float = 0.1, dictionary_id: int = cv2.aruco.DICT_4X4_1000):
        """
        Initialize marker placement.
        
        Args:
            maze: The maze
            marker_size: Size of markers in meters (default: 0.1)
            dictionary_id: ArUco dictionary ID (default: DICT_4X4_1000)
        """
        self.maze = maze
        self.marker_size = marker_size
        self.gap = 0.5 * marker_size  # Gap between markers
        self.dictionary = cv2.aruco.getPredefinedDictionary(dictionary_id)
        
        # Store marker positions: {marker_id: (x, y, yaw, quadrant)}
        # Quadrant: 0=top-left, 1=top-right, 2=bottom-left, 3=bottom-right
        self.markers: Dict[int, Tuple[float, float, float, int]] = {}
        
        self._place_markers()
    
    def _place_markers(self) -> None:
        """Place markers for each cell in a 2x2 pattern on the floor."""
        for r in range(self.maze.rows):
            for c in range(self.maze.cols):
                # Get cell center
                cell_x, cell_y = self.maze.cell_to_world_center(r, c)
                
                # Calculate offsets for 2x2 pattern
                # Total footprint = 2*markerSize + gap
                # Marker centers at +/- (markerSize/2 + gap/2)
                offset = self.marker_size / 2.0 + self.gap / 2.0
                
                # Place 4 markers
                # Quadrant 0: top-left (relative to cell center)
                marker_id_0 = self._encode_id(r, c, 0)
                self.markers[marker_id_0] = (
                    cell_x - offset,
                    cell_y - offset,
                    0.0,  # yaw = 0 (facing up)
                    0
                )
                
                # Quadrant 1: top-right
                marker_id_1 = self._encode_id(r, c, 1)
                self.markers[marker_id_1] = (
                    cell_x + offset,
                    cell_y - offset,
                    0.0,
                    1
                )
                
                # Quadrant 2: bottom-left
                marker_id_2 = self._encode_id(r, c, 2)
                self.markers[marker_id_2] = (
                    cell_x - offset,
                    cell_y + offset,
                    0.0,
                    2
                )
                
                # Quadrant 3: bottom-right
                marker_id_3 = self._encode_id(r, c, 3)
                self.markers[marker_id_3] = (
                    cell_x + offset,
                    cell_y + offset,
                    0.0,
                    3
                )
    
    def _encode_id(self, r: int, c: int, quadrant: int) -> int:
        """
        Encode marker ID from cell position and quadrant.
        
        Args:
            r: Row index
            c: Column index
            quadrant: 0=top-left, 1=top-right, 2=bottom-left, 3=bottom-right
        
        Returns:
            Marker ID
        """
        base = r * self.maze.cols + c
        return base * 10 + quadrant
    
    def decode_id(self, marker_id: int) -> Optional[Tuple[int, int, int]]:
        """
        Decode marker ID to cell position and quadrant.
        
        Args:
            marker_id: The marker ID
        
        Returns:
            Tuple (row, col, quadrant) or None if invalid
        """
        quadrant = marker_id % 10
        base = marker_id // 10
        
        if quadrant not in [0, 1, 2, 3]:
            return None
        
        r = base // self.maze.cols
        c = base % self.maze.cols
        
        if 0 <= r < self.maze.rows and 0 <= c < self.maze.cols:
            return (r, c, quadrant)
        
        return None
    
    def get_marker_image(self, marker_id: int, size_pixels: int = 200) -> np.ndarray:
        """
        Generate ArUco marker image.
        
        Args:
            marker_id: The marker ID
            size_pixels: Size of marker image in pixels
        
        Returns:
            Grayscale marker image
        """
        # OpenCV API compatibility: generateImageMarker may not exist in all builds
        if hasattr(cv2.aruco, "generateImageMarker"):
            marker_img = cv2.aruco.generateImageMarker(
                self.dictionary,
                marker_id,
                size_pixels
            )
        else:
            marker_img = np.zeros((size_pixels, size_pixels), dtype=np.uint8)
            cv2.aruco.drawMarker(self.dictionary, marker_id, size_pixels, marker_img, 1)
        return marker_img
    
    def get_markers_in_range(
        self,
        x: float,
        y: float,
        z: float,
        max_range: float = 2.0
    ) -> List[Tuple[int, float, float, float, int]]:
        """
        Get markers within range of a position (for camera simulation).
        
        Args:
            x: Observer X position
            y: Observer Y position
            z: Observer Z position (height)
            max_range: Maximum detection range in meters
        
        Returns:
            List of (marker_id, marker_x, marker_y, marker_yaw, quadrant)
        """
        visible_markers = []
        
        for marker_id, (mx, my, myaw, quadrant) in self.markers.items():
            # Calculate horizontal distance
            dx = mx - x
            dy = my - y
            distance = math.sqrt(dx**2 + dy**2)
            
            # Check if in range
            if distance <= max_range:
                visible_markers.append((marker_id, mx, my, myaw, quadrant))
        
        return visible_markers
