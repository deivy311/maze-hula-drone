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
    
    def __init__(self, maze: Maze, marker_size: float = None, dictionary_id: int = cv2.aruco.DICT_4X4_1000):
        """
        Initialize marker placement.
        
        Args:
            maze: The maze
            marker_size: Size of markers in meters. If None, calculated to fit cell with 
                        border distance = marker_size/2 (default: None)
            dictionary_id: ArUco dictionary ID (default: DICT_4X4_1000)
        """
        self.maze = maze
        # Markers are placed in a uniform grid
        # Gap between markers = marker_size/2
        # Marker spacing (center to center) = marker_size + gap = 1.5*marker_size
        # Each cell has 2 markers in each direction
        # Cell size = 2 * marker_spacing = 2 * 1.5*marker_size = 3*marker_size
        # Therefore: marker_size = cell_size/3
        # But we can also calculate cell_size from marker_size if needed
        max_marker_size = maze.cell_size / 3.0
        
        if marker_size is None:
            # Calculate optimal size based on constraint
            self.marker_size = max_marker_size
        else:
            # Use specified size, but cap at maximum that allows constraint
            self.marker_size = min(marker_size, max_marker_size)
        
        # Calculate gap based on layout: 3*gap + 2*marker_size = cell_size
        # gap = (cell_size - 2*marker_size) / 3
        # With marker_size = cell_size/4: gap = (cell_size - cell_size/2) / 3 = cell_size/6
        # But user wants gap = marker_size/2, so let's use that
        # This means marker_size will be adjusted if needed
        self.gap = 0.5 * self.marker_size  # Gap between markers = marker_size/2
        self.dictionary = cv2.aruco.getPredefinedDictionary(dictionary_id)
        
        # Store marker positions: {marker_id: (x, y, yaw, quadrant)}
        # Quadrant: 0=top-left, 1=top-right, 2=bottom-left, 3=bottom-right
        self.markers: Dict[int, Tuple[float, float, float, int]] = {}
        
        self._place_markers()
    
    def _place_markers(self) -> None:
        """Place markers in a uniform grid.
        
        Markers are placed in a uniform grid where:
        - Gap between adjacent markers = marker_size/2
        - Each cell has 4 markers in a 2x2 pattern
        - Markers are evenly spaced across all cells
        - Walls will be placed in the middle between adjacent markers
        """
        # Gap between marker centers = marker_size + gap = marker_size + marker_size/2 = 1.5*marker_size
        marker_spacing = self.marker_size + self.gap
        
        # For R x C cells, we have (2*R) x (2*C) markers in total
        # Start position: first marker at (marker_size/2, marker_size/2) from origin
        # This ensures gap = marker_size/2 from edges
        start_offset = self.marker_size / 2.0 + self.gap
        
        for r in range(self.maze.rows):
            for c in range(self.maze.cols):
                # Calculate marker positions in uniform grid
                # Each cell has 4 markers at positions:
                # (2*c, 2*r), (2*c+1, 2*r), (2*c, 2*r+1), (2*c+1, 2*r+1) in marker grid
                
                # Top-left marker of cell (r, c)
                grid_x_0 = 2 * c
                grid_y_0 = 2 * r
                marker_id_0 = self._encode_id(r, c, 0)
                self.markers[marker_id_0] = (
                    start_offset + grid_x_0 * marker_spacing,
                    start_offset + grid_y_0 * marker_spacing,
                    0.0,
                    0
                )
                
                # Top-right marker
                grid_x_1 = 2 * c + 1
                grid_y_1 = 2 * r
                marker_id_1 = self._encode_id(r, c, 1)
                self.markers[marker_id_1] = (
                    start_offset + grid_x_1 * marker_spacing,
                    start_offset + grid_y_1 * marker_spacing,
                    0.0,
                    1
                )
                
                # Bottom-left marker
                grid_x_2 = 2 * c
                grid_y_2 = 2 * r + 1
                marker_id_2 = self._encode_id(r, c, 2)
                self.markers[marker_id_2] = (
                    start_offset + grid_x_2 * marker_spacing,
                    start_offset + grid_y_2 * marker_spacing,
                    0.0,
                    2
                )
                
                # Bottom-right marker
                grid_x_3 = 2 * c + 1
                grid_y_3 = 2 * r + 1
                marker_id_3 = self._encode_id(r, c, 3)
                self.markers[marker_id_3] = (
                    start_offset + grid_x_3 * marker_spacing,
                    start_offset + grid_y_3 * marker_spacing,
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
