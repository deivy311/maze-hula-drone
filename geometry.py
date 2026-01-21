"""
Geometric representation of maze walls and columns with collision detection.
"""

import math
from typing import List, Tuple
import numpy as np


class WallSegment:
    """Represents a wall segment between two columns."""
    
    def __init__(self, x1: float, y1: float, x2: float, y2: float, thickness: float):
        """
        Initialize wall segment.
        
        Args:
            x1, y1: Start point
            x2, y2: End point
            thickness: Wall thickness in meters
        """
        self.x1 = x1
        self.y1 = y1
        self.x2 = x2
        self.y2 = y2
        self.thickness = thickness
        
        # Precompute for collision checking
        self.length = math.sqrt((x2 - x1)**2 + (y2 - y1)**2)
        if self.length > 0:
            self.dx = (x2 - x1) / self.length
            self.dy = (y2 - y1) / self.length
            # Perpendicular direction (normalized)
            self.nx = -self.dy
            self.ny = self.dx
        else:
            self.dx = 0
            self.dy = 0
            self.nx = 0
            self.ny = 1
    
    def check_collision_circle(self, cx: float, cy: float, radius: float) -> bool:
        """
        Check if a circle collides with this wall segment.
        
        Args:
            cx, cy: Circle center
            radius: Circle radius
        
        Returns:
            True if collision detected
        """
        # Vector from segment start to circle center
        vx = cx - self.x1
        vy = cy - self.y1
        
        # Project onto segment direction
        proj = vx * self.dx + vy * self.dy
        
        # Clamp to segment bounds
        proj = max(0, min(self.length, proj))
        
        # Closest point on segment
        closest_x = self.x1 + proj * self.dx
        closest_y = self.y1 + proj * self.dy
        
        # Distance from circle center to closest point
        dist = math.sqrt((cx - closest_x)**2 + (cy - closest_y)**2)
        
        # Check if within radius + half thickness
        return dist < (radius + self.thickness / 2.0)


class Column:
    """Represents a column (circular obstacle) at a grid vertex."""
    
    def __init__(self, x: float, y: float, radius: float):
        """
        Initialize column.
        
        Args:
            x, y: Column center position
            radius: Column radius in meters
        """
        self.x = x
        self.y = y
        self.radius = radius
    
    def check_collision_circle(self, cx: float, cy: float, radius: float) -> bool:
        """
        Check if a circle collides with this column.
        
        Args:
            cx, cy: Circle center
            radius: Circle radius
        
        Returns:
            True if collision detected
        """
        dist = math.sqrt((cx - self.x)**2 + (cy - self.y)**2)
        return dist < (self.radius + radius)


class MazeGeometry:
    """Geometric representation of maze with walls and columns."""
    
    def __init__(
        self,
        rows: int,
        cols: int,
        cell_size: float,
        wall_thickness: float,
        col_radius: float,
        walls: List[List[dict]]
    ):
        """
        Initialize maze geometry.
        
        Args:
            rows: Number of rows
            cols: Number of columns
            cell_size: Size of each cell in meters
            wall_thickness: Thickness of wall segments
            col_radius: Radius of columns
            walls: Wall data from maze (walls[r][c] = {'N': bool, ...})
        """
        self.rows = rows
        self.cols = cols
        self.cell_size = cell_size
        self.wall_thickness = wall_thickness
        self.col_radius = col_radius
        
        # Generate columns at all grid vertices
        self.columns: List[Column] = []
        for r in range(rows + 1):
            for c in range(cols + 1):
                x = c * cell_size
                y = r * cell_size
                self.columns.append(Column(x, y, col_radius))
        
        # Generate wall segments
        self.wall_segments: List[WallSegment] = []
        
        # Horizontal walls (between rows)
        for r in range(rows + 1):
            for c in range(cols):
                # Check if wall exists
                has_wall = False
                if r == 0:
                    # Top boundary
                    has_wall = True
                elif r == rows:
                    # Bottom boundary
                    has_wall = True
                else:
                    # Internal wall: check cell above
                    has_wall = walls[r - 1][c]['S']
                
                if has_wall:
                    x1 = c * cell_size
                    y1 = r * cell_size
                    x2 = (c + 1) * cell_size
                    y2 = r * cell_size
                    self.wall_segments.append(WallSegment(x1, y1, x2, y2, wall_thickness))
        
        # Vertical walls (between columns)
        for c in range(cols + 1):
            for r in range(rows):
                # Check if wall exists
                has_wall = False
                if c == 0:
                    # Left boundary
                    has_wall = True
                elif c == cols:
                    # Right boundary
                    has_wall = True
                else:
                    # Internal wall: check cell to left
                    has_wall = walls[r][c - 1]['E']
                
                if has_wall:
                    x1 = c * cell_size
                    y1 = r * cell_size
                    x2 = c * cell_size
                    y2 = (r + 1) * cell_size
                    self.wall_segments.append(WallSegment(x1, y1, x2, y2, wall_thickness))
    
    def check_collision(self, x: float, y: float, radius: float) -> bool:
        """
        Check if a circle at position (x, y) collides with walls or columns.
        
        Args:
            x, y: Position in meters
            radius: Circle radius in meters
        
        Returns:
            True if collision detected
        """
        # Check bounds
        if x < 0 or x > self.cols * self.cell_size:
            return True
        if y < 0 or y > self.rows * self.cell_size:
            return True
        
        # Check columns
        for column in self.columns:
            if column.check_collision_circle(x, y, radius):
                return True
        
        # Check wall segments
        for wall in self.wall_segments:
            if wall.check_collision_circle(x, y, radius):
                return True
        
        return False
    
    def clamp_position(self, x: float, y: float, radius: float) -> Tuple[float, float]:
        """
        Clamp position to avoid collisions (simple approach: push away from obstacles).
        
        Args:
            x, y: Desired position
            radius: Circle radius
        
        Returns:
            Clamped (x, y) position
        """
        # Simple approach: if collision, try to move away
        if not self.check_collision(x, y, radius):
            return (x, y)
        
        # Try small adjustments
        for dx in [-0.01, 0, 0.01]:
            for dy in [-0.01, 0, 0.01]:
                new_x = x + dx
                new_y = y + dy
                if not self.check_collision(new_x, new_y, radius):
                    return (new_x, new_y)
        
        # If still colliding, return original (reject movement)
        return (x, y)
