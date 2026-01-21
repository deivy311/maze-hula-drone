"""
Maze generation module using randomized DFS/backtracking algorithm.
"""

import random
from typing import List, Tuple, Optional
from geometry import MazeGeometry


class Maze:
    """Represents a perfect maze with walls between cells."""
    
    def __init__(
        self,
        rows: int,
        cols: int,
        cell_size: float = 1.0,
        wall_thickness: float = 0.02,
        col_radius: float = 0.05,
        seed: Optional[int] = None
    ):
        """
        Initialize a maze grid.
        
        Args:
            rows: Number of rows in the maze
            cols: Number of columns in the maze
            cell_size: Size of each cell in meters (default: 1.0)
            wall_thickness: Thickness of wall segments in meters
            col_radius: Radius of columns in meters
            seed: Optional random seed for reproducibility
        """
        if rows < 1 or cols < 1:
            raise ValueError("Rows and cols must be positive integers")
        if cell_size <= 0:
            raise ValueError("Cell size must be positive")
        
        self.rows = rows
        self.cols = cols
        self.cell_size = cell_size
        self.wall_thickness = wall_thickness
        self.col_radius = col_radius
        
        if seed is not None:
            random.seed(seed)
        
        # Each cell has 4 walls: N, E, S, W
        # walls[r][c] = {'N': bool, 'E': bool, 'S': bool, 'W': bool}
        # True means wall exists, False means passage is open
        self.walls: List[List[dict]] = [
            [{'N': True, 'E': True, 'S': True, 'W': True} 
             for _ in range(cols)] 
            for _ in range(rows)
        ]
        
        self._generate()
        
        # Create geometry
        self.geometry = MazeGeometry(
            rows, cols, cell_size, wall_thickness, col_radius, self.walls
        )
    
    def _generate(self) -> None:
        """Generate a perfect maze using randomized DFS/backtracking."""
        visited = [[False for _ in range(self.cols)] for _ in range(self.rows)]
        stack: List[Tuple[int, int]] = []
        
        # Start from (0, 0)
        start_r, start_c = 0, 0
        stack.append((start_r, start_c))
        visited[start_r][start_c] = True
        
        # Directions: (dr, dc, wall_side, opposite_wall_side)
        directions = [
            (-1, 0, 'N', 'S'),  # North
            (0, 1, 'E', 'W'),   # East
            (1, 0, 'S', 'N'),   # South
            (0, -1, 'W', 'E'),  # West
        ]
        
        while stack:
            current_r, current_c = stack[-1]
            
            # Find unvisited neighbors
            neighbors: List[Tuple[int, int, str, str]] = []
            for dr, dc, wall_side, opp_wall_side in directions:
                nr, nc = current_r + dr, current_c + dc
                if (0 <= nr < self.rows and 
                    0 <= nc < self.cols and 
                    not visited[nr][nc]):
                    neighbors.append((nr, nc, wall_side, opp_wall_side))
            
            if neighbors:
                # Choose random unvisited neighbor
                nr, nc, wall_side, opp_wall_side = random.choice(neighbors)
                
                # Remove wall between current and neighbor
                self.walls[current_r][current_c][wall_side] = False
                self.walls[nr][nc][opp_wall_side] = False
                
                # Mark neighbor as visited and add to stack
                visited[nr][nc] = True
                stack.append((nr, nc))
            else:
                # Backtrack
                stack.pop()
    
    def has_wall(self, r: int, c: int, side: str) -> bool:
        """
        Check if a wall exists at the given cell and side.
        
        Args:
            r: Row index
            c: Column index
            side: One of 'N', 'E', 'S', 'W'
        
        Returns:
            True if wall exists, False if passage is open
        """
        if not (0 <= r < self.rows and 0 <= c < self.cols):
            raise IndexError(f"Cell ({r}, {c}) is out of bounds")
        if side not in ['N', 'E', 'S', 'W']:
            raise ValueError(f"Side must be one of 'N', 'E', 'S', 'W', got {side}")
        
        return self.walls[r][c][side]
    
    def world_to_cell(self, x: float, y: float) -> Tuple[int, int]:
        """
        Convert continuous world coordinates to cell indices.
        
        Args:
            x: X coordinate in meters
            y: Y coordinate in meters
        
        Returns:
            Tuple (row, col) cell indices
        """
        c = int(x / self.cell_size)
        r = int(y / self.cell_size)
        # Clamp to valid range
        c = max(0, min(self.cols - 1, c))
        r = max(0, min(self.rows - 1, r))
        return (r, c)
    
    def cell_to_world_center(self, r: int, c: int) -> Tuple[float, float]:
        """
        Convert cell indices to world coordinates (cell center).
        
        Args:
            r: Row index
            c: Column index
        
        Returns:
            Tuple (x, y) in meters
        """
        x = (c + 0.5) * self.cell_size
        y = (r + 0.5) * self.cell_size
        return (x, y)
    
    def check_collision(self, x: float, y: float, radius: float) -> bool:
        """
        Check if a position collides with walls or columns.
        
        Args:
            x: X coordinate in meters
            y: Y coordinate in meters
            radius: Collision radius in meters
        
        Returns:
            True if collision detected, False otherwise
        """
        return self.geometry.check_collision(x, y, radius)
