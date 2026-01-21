#!/usr/bin/env python3
"""
Main simulation runner for drone maze navigation.
"""

import argparse
import sys
from typing import Optional
import numpy as np
import matplotlib
# Set backend before importing pyplot
# Try different backends in order of preference
backend_set = False
for backend_name in ['TkAgg', 'Qt5Agg', 'QtAgg']:
    try:
        matplotlib.use(backend_name)
        backend_set = True
        break
    except:
        continue

# If no GUI backend available, check if we're in a headless environment
if not backend_set:
    import os
    if 'DISPLAY' not in os.environ:
        # Headless environment - use Agg but warn user
        print("Warning: No GUI backend available. Visualization may not work properly.")
        print("Install tkinter or run with X11 forwarding if using SSH.")
    # Use default backend

import matplotlib.pyplot as plt
try:
    plt.ion()  # Enable interactive mode
except:
    pass  # May fail in some environments
import cv2

from maze import Maze
from drone import Drone, CellToCellController
from markers import MarkerPlacement
from camera_sim import CameraSim
from detect import MarkerDetector


class Simulation:
    """Main simulation loop."""
    
    def __init__(
        self,
        rows: int = 5,
        cols: int = 5,
        cell_size: float = 1.0,
        marker_size: float = 0.1,
        col_radius: float = 0.05,
        wall_thickness: float = 0.02,
        drone_radius: float = 0.1,
        z: float = 1.0,
        dt: float = 1.0 / 30.0,
        duration: float = 60.0,
        seed: Optional[int] = None,
        show: bool = True
    ):
        """
        Initialize simulation.
        
        Args:
            rows: Maze rows
            cols: Maze columns
            cell_size: Size of each cell in meters
            marker_size: Size of ArUco markers in meters
            col_radius: Radius of columns in meters
            wall_thickness: Thickness of wall segments in meters
            drone_radius: Drone footprint radius in meters
            z: Drone altitude in meters
            dt: Time step in seconds
            duration: Simulation duration in seconds
            seed: Random seed
            show: Whether to show visualization
        """
        self.dt = dt
        self.duration = duration
        self.show = show
        
        # Create maze
        print(f"Generating {rows}x{cols} maze...")
        self.maze = Maze(
            rows, cols,
            cell_size=cell_size,
            wall_thickness=wall_thickness,
            col_radius=col_radius,
            seed=seed
        )
        
        # Create marker placement
        print("Placing ArUco markers...")
        self.marker_placement = MarkerPlacement(self.maze, marker_size=marker_size)
        
        # Create drone at start position
        start_x, start_y = self.maze.cell_to_world_center(0, 0)
        self.drone = Drone(x=start_x, y=start_y, yaw=0.0, z=z, radius=drone_radius)
        
        # Create controller
        self.controller = CellToCellController(self.maze)
        
        # Create camera
        self.camera = CameraSim(width=640, height=480, fx=500.0, fy=500.0)
        
        # Create detector
        self.detector = MarkerDetector()
        
        # Logging
        self.log: list = []
        self.detection_count = 0
        self.total_frames = 0
        
        # Visualization
        if self.show:
            self._setup_visualization()
    
    def _setup_visualization(self) -> None:
        """Setup matplotlib visualization."""
        self.fig = plt.figure(figsize=(14, 6))
        self.fig.show()  # Show the figure window
        
        # Top-down map view
        self.ax_map = self.fig.add_subplot(1, 2, 1)
        self.ax_map.set_aspect('equal')
        self.ax_map.set_xlim(-0.5, self.maze.cols * self.maze.cell_size + 0.5)
        self.ax_map.set_ylim(-0.5, self.maze.rows * self.maze.cell_size + 0.5)
        self.ax_map.set_xlabel('X (m)')
        self.ax_map.set_ylabel('Y (m)')
        self.ax_map.set_title('Top-Down View')
        self.ax_map.invert_yaxis()  # Y-axis down for image-like coordinates
        
        # Camera view
        self.ax_cam = self.fig.add_subplot(1, 2, 2)
        self.ax_cam.set_title('Camera View')
        self.ax_cam.axis('off')
        
        # Draw maze walls and columns
        self._draw_maze()
        
        # Drone position marker
        self.drone_marker, = self.ax_map.plot([], [], 'ro', markersize=10, label='Drone')
        self.drone_arrow = self.ax_map.annotate('', xy=(0, 0), xytext=(0, 0),
                                                  arrowprops=dict(arrowstyle='->', color='red', lw=2))
        
        self.ax_map.legend()
        plt.tight_layout()
    
    def _draw_maze(self) -> None:
        """Draw maze walls and columns on map."""
        # Draw columns
        for col in self.maze.geometry.columns:
            circle = plt.Circle((col.x, col.y), col.radius, color='gray', zorder=3)
            self.ax_map.add_patch(circle)
        
        # Draw wall segments
        for wall in self.maze.geometry.wall_segments:
            # Draw wall as a thick line
            self.ax_map.plot(
                [wall.x1, wall.x2],
                [wall.y1, wall.y2],
                'k-',
                linewidth=self.maze.wall_thickness * 50,  # Scale for visibility
                zorder=2
            )
    
    def _update_visualization(self) -> None:
        """Update visualization."""
        if not self.show:
            return
        
        # Update drone position
        x, y, yaw = self.drone.get_position()
        self.drone_marker.set_data([x], [y])
        
        # Update arrow
        arrow_len = 0.3
        dx = arrow_len * np.cos(yaw)
        dy = arrow_len * np.sin(yaw)
        self.drone_arrow.set_position((x, y))
        self.drone_arrow.xy = (x + dx, y + dy)
        
        # Update camera view
        frame = self.camera.generate_frame(
            x, y, self.drone.z, yaw,
            self.marker_placement,
            marker_size=self.marker_placement.marker_size
        )
        
        # Detect markers and draw
        marker_ids, marker_corners, _ = self.detector.detect(frame)
        frame_display = frame.copy()
        
        if len(marker_ids) > 0:
            cv2.aruco.drawDetectedMarkers(
                frame_display,
                [np.array([c]) for c in marker_corners],
                np.array(marker_ids)
            )
        
        self.ax_cam.clear()
        self.ax_cam.imshow(frame_display)
        self.ax_cam.set_title('Camera View')
        self.ax_cam.axis('off')
        
        self.fig.canvas.draw()
        self.fig.canvas.flush_events()
        plt.pause(0.01)
    
    def run(self) -> None:
        """Run simulation loop."""
        print(f"Starting simulation for {self.duration} seconds at {1.0/self.dt:.1f} Hz...")
        print(f"Drone at ({self.drone.x:.2f}, {self.drone.y:.2f}), altitude {self.drone.z:.2f}m")
        
        t = 0.0
        step = 0
        print_interval = max(1, int(1.0 / self.dt))  # Print once per second
        
        while t < self.duration:
            # Update drone state
            success = self.drone.update(self.dt, self.maze)
            if not success:
                print(f"Collision detected at t={t:.2f}s!")
                break
            
            # Generate camera frame
            x, y, yaw = self.drone.get_position()
            frame = self.camera.generate_frame(
                x, y, self.drone.z, yaw,
                self.marker_placement,
                marker_size=self.marker_placement.marker_size
            )
            
            # Detect markers
            marker_ids, marker_corners, _ = self.detector.detect(frame)
            self.total_frames += 1
            
            # Decode detections
            detected_cells = []
            for marker_id in marker_ids:
                decoded = self.marker_placement.decode_id(marker_id)
                if decoded is not None:
                    r, c, quadrant = decoded
                    detected_cells.append((r, c, quadrant))
                    self.detection_count += 1
            
            # Get most common detected cell (simple strategy)
            detected_cell = None
            if detected_cells:
                # Use first detection for now (could use voting)
                r, c, quadrant = detected_cells[0]
                detected_cell = (r, c)
            
            # Update controller
            self.controller.update(self.drone, detected_cell)
            
            # Log
            current_cell = self.drone.get_cell(self.maze)
            log_entry = {
                't': t,
                'x': x,
                'y': y,
                'yaw': yaw,
                'cell': current_cell,
                'detected_markers': marker_ids,
                'detected_cells': detected_cells
            }
            self.log.append(log_entry)
            
            # Print status
            if step % print_interval == 0:
                detection_rate = (self.detection_count / self.total_frames * 100) if self.total_frames > 0 else 0
                print(f"t={t:.2f}s: pos=({x:.2f}, {y:.2f}), cell={current_cell}, "
                      f"detected={len(marker_ids)} markers, rate={detection_rate:.1f}%")
            
            # Update visualization
            if self.show:
                self._update_visualization()
            
            t += self.dt
            step += 1
        
        print(f"\nSimulation complete. Total steps: {step}")
        print(f"Final position: ({x:.2f}, {y:.2f}), cell: {current_cell}")
        if self.total_frames > 0:
            detection_rate = self.detection_count / self.total_frames * 100
            print(f"Detection rate: {detection_rate:.1f}% ({self.detection_count}/{self.total_frames} frames)")
        
        if self.show:
            plt.show(block=True)


def main():
    """Main CLI function."""
    parser = argparse.ArgumentParser(
        description='Drone maze navigation simulation with ArUco markers',
        formatter_class=argparse.RawDescriptionHelpFormatter
    )
    
    parser.add_argument('--rows', type=int, default=5, help='Maze rows (default: 5)')
    parser.add_argument('--cols', type=int, default=5, help='Maze columns (default: 5)')
    parser.add_argument('--cellSize', type=float, default=1.0, help='Cell size in meters (default: 1.0)')
    parser.add_argument('--markerSize', type=float, default=0.1, help='Marker size in meters (default: 0.1)')
    parser.add_argument('--colRadius', type=float, default=0.05, help='Column radius in meters (default: 0.05)')
    parser.add_argument('--wallThickness', type=float, default=0.02, help='Wall thickness in meters (default: 0.02)')
    parser.add_argument('--droneRadius', type=float, default=0.1, help='Drone radius in meters (default: 0.1)')
    parser.add_argument('--z', type=float, default=1.0, help='Drone altitude in meters (default: 1.0)')
    parser.add_argument('--seed', type=int, default=None, help='Random seed (optional)')
    parser.add_argument('--seconds', type=float, default=60.0, help='Simulation duration in seconds (default: 60)')
    parser.add_argument('--show', action='store_true', default=True, help='Show visualization (default: True)')
    parser.add_argument('--no-show', dest='show', action='store_false', help='Disable visualization')
    
    args = parser.parse_args()
    
    # Validate inputs
    if args.rows < 1 or args.cols < 1:
        print("Error: rows and cols must be positive", file=sys.stderr)
        sys.exit(1)
    
    if args.cellSize <= 0 or args.markerSize <= 0 or args.colRadius <= 0:
        print("Error: sizes must be positive", file=sys.stderr)
        sys.exit(1)
    
    if args.wallThickness <= 0 or args.droneRadius <= 0 or args.z <= 0:
        print("Error: thickness, radius, and z must be positive", file=sys.stderr)
        sys.exit(1)
    
    if args.seconds <= 0:
        print("Error: seconds must be positive", file=sys.stderr)
        sys.exit(1)
    
    try:
        sim = Simulation(
            rows=args.rows,
            cols=args.cols,
            cell_size=args.cellSize,
            marker_size=args.markerSize,
            col_radius=args.colRadius,
            wall_thickness=args.wallThickness,
            drone_radius=args.droneRadius,
            z=args.z,
            dt=1.0/30.0,  # 30 Hz
            duration=args.seconds,
            seed=args.seed,
            show=args.show
        )
        sim.run()
    except KeyboardInterrupt:
        print("\nSimulation interrupted by user")
        sys.exit(0)
    except Exception as e:
        print(f"Error: {e}", file=sys.stderr)
        import traceback
        traceback.print_exc()
        sys.exit(1)


if __name__ == '__main__':
    main()
