"""
Drone simulation module with kinematic state and control.
"""

import math
from typing import Tuple, Optional
from maze import Maze


class Drone:
    """Represents a drone with kinematic state."""
    
    def __init__(
        self,
        x: float = 0.0,
        y: float = 0.0,
        yaw: float = 0.0,
        z: float = 1.0,
        radius: float = 0.1
    ):
        """
        Initialize drone state.
        
        Args:
            x: Initial X position in meters
            y: Initial Y position in meters
            yaw: Initial yaw angle in radians (0 = +X direction)
            z: Fixed altitude in meters (default: 1.0)
            radius: Drone footprint radius in meters (default: 0.1)
        """
        self.x = x
        self.y = y
        self.yaw = yaw
        self.z = z
        self.radius = radius
        
        # Current velocity commands
        self.v = 0.0  # Forward velocity (m/s)
        self.omega = 0.0  # Angular velocity (rad/s)
    
    def set_velocity(self, v: float, omega: float) -> None:
        """
        Set velocity commands.
        
        Args:
            v: Forward velocity in m/s
            omega: Angular velocity in rad/s
        """
        self.v = v
        self.omega = omega
    
    def update(self, dt: float, maze: Optional[Maze] = None) -> bool:
        """
        Update drone state using kinematic model with collision checking.
        
        Args:
            dt: Time step in seconds
            maze: Optional maze for collision checking
        
        Returns:
            True if update successful (no collision), False if collision detected
        """
        # Update yaw
        new_yaw = self.yaw + self.omega * dt
        
        # Normalize yaw to [0, 2*pi)
        new_yaw = new_yaw % (2 * math.pi)
        
        # Update position
        new_x = self.x + self.v * math.cos(self.yaw) * dt
        new_y = self.y + self.v * math.sin(self.yaw) * dt
        
        # Check collision if maze provided
        if maze is not None:
            if maze.check_collision(new_x, new_y, self.radius):
                # Clamp position to avoid collision
                clamped_x, clamped_y = maze.geometry.clamp_position(new_x, new_y, self.radius)
                # If still colliding, reject movement
                if maze.check_collision(clamped_x, clamped_y, self.radius):
                    return False
                new_x, new_y = clamped_x, clamped_y
        
        # Apply update
        self.x = new_x
        self.y = new_y
        self.yaw = new_yaw
        
        return True
    
    def get_position(self) -> Tuple[float, float, float]:
        """
        Get current position.
        
        Returns:
            Tuple (x, y, yaw) in meters and radians
        """
        return (self.x, self.y, self.yaw)
    
    def get_cell(self, maze: Maze) -> Tuple[int, int]:
        """
        Get current cell indices.
        
        Args:
            maze: The maze to query
        
        Returns:
            Tuple (row, col) cell indices
        """
        return maze.world_to_cell(self.x, self.y)


class CellToCellController:
    """Simple controller that drives the drone cell-to-cell using marker detections."""
    
    def __init__(self, maze: Maze, target_cell: Optional[Tuple[int, int]] = None):
        """
        Initialize controller.
        
        Args:
            maze: The maze
            target_cell: Optional target cell (row, col). If None, will navigate to end.
        """
        self.maze = maze
        self.target_cell = target_cell
        if target_cell is None:
            # Default to bottom-right corner
            self.target_cell = (maze.rows - 1, maze.cols - 1)
        
        self.current_target: Optional[Tuple[int, int]] = None
        self.reached_target = False
    
    def update(
        self,
        drone: Drone,
        detected_cell: Optional[Tuple[int, int]],
        v_max: float = 0.5,
        omega_max: float = 1.0
    ) -> None:
        """
        Update controller based on current state and detections.
        
        Args:
            drone: The drone to control
            detected_cell: Currently detected cell from markers (row, col)
            v_max: Maximum forward velocity (m/s)
            omega_max: Maximum angular velocity (rad/s)
        """
        if detected_cell is None:
            # No detection, stop
            drone.set_velocity(0.0, 0.0)
            return
        
        current_cell = drone.get_cell(self.maze)
        
        # Simple strategy: move toward target cell
        if current_cell == self.target_cell:
            drone.set_velocity(0.0, 0.0)
            self.reached_target = True
            return
        
        # Get target cell center
        target_x, target_y = self.maze.cell_to_world_center(*self.target_cell)
        
        # Calculate desired heading
        dx = target_x - drone.x
        dy = target_y - drone.y
        desired_yaw = math.atan2(dy, dx)
        
        # Calculate yaw error
        yaw_error = desired_yaw - drone.yaw
        
        # Normalize to [-pi, pi]
        while yaw_error > math.pi:
            yaw_error -= 2 * math.pi
        while yaw_error < -math.pi:
            yaw_error += 2 * math.pi
        
        # Control: turn toward target, then move forward
        if abs(yaw_error) > 0.1:  # 0.1 rad ~ 6 degrees
            # Turn in place
            omega = omega_max * math.copysign(1, yaw_error)
            drone.set_velocity(0.0, omega)
        else:
            # Move forward
            distance = math.sqrt(dx**2 + dy**2)
            if distance > 0.1:  # 0.1 m threshold
                drone.set_velocity(v_max, 0.0)
            else:
                drone.set_velocity(0.0, 0.0)
