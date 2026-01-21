#!/usr/bin/env python3
"""
Main simulation runner for Gazebo-based drone maze navigation.
"""

import argparse
import sys
import time
from typing import Optional
import numpy as np
import cv2

from maze import Maze
from drone import CellToCellController
from markers import MarkerPlacement
from detect import MarkerDetector

try:
    from gazebo_interface import GazeboInterface
    ROS_AVAILABLE = True
except ImportError:
    ROS_AVAILABLE = False
    print("Warning: ROS not available. Install ROS packages for full functionality.")


class GazeboSimulation:
    """Gazebo-based simulation."""
    
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
        seed: Optional[int] = None,
        duration: float = 60.0
    ):
        """
        Initialize Gazebo simulation.
        
        Args:
            rows: Maze rows
            cols: Maze columns
            cell_size: Cell size in meters
            marker_size: Marker size in meters
            col_radius: Column radius
            wall_thickness: Wall thickness
            drone_radius: Drone radius
            z: Drone altitude
            seed: Random seed
            duration: Simulation duration
        """
        if not ROS_AVAILABLE:
            raise RuntimeError("ROS is required for Gazebo simulation. Please install ROS packages.")
        
        self.duration = duration
        
        # Generate maze
        print(f"Generating {rows}x{cols} maze...")
        self.maze = Maze(
            rows, cols,
            cell_size=cell_size,
            wall_thickness=wall_thickness,
            col_radius=col_radius,
            seed=seed
        )
        
        # Generate markers
        print("Placing ArUco markers...")
        self.marker_placement = MarkerPlacement(self.maze, marker_size=marker_size)
        
        # Generate Gazebo world
        print("Generating Gazebo world file...")
        from gazebo_gen import generate_gazebo_world
        world_file = "gazebo/worlds/generated_maze.world"
        generate_gazebo_world(self.maze, self.marker_placement, world_file)
        print(f"World file generated: {world_file}")
        print("Please launch Gazebo with: gazebo " + world_file)
        print("Waiting for Gazebo to start...")
        
        # Wait a bit for user to start Gazebo
        time.sleep(5)
        
        # Connect to Gazebo
        print("Connecting to Gazebo...")
        self.interface = GazeboInterface(self.marker_placement)
        
        # Create controller
        self.controller = CellToCellController(self.maze)
        
        # Logging
        self.log = []
        self.detection_count = 0
        self.total_frames = 0
    
    def run(self) -> None:
        """Run simulation loop."""
        import rospy
        
        print(f"Starting simulation for {self.duration} seconds...")
        
        rate = rospy.Rate(30)  # 30 Hz
        start_time = rospy.Time.now()
        
        while not rospy.is_shutdown():
            current_time = rospy.Time.now()
            elapsed = (current_time - start_time).to_sec()
            
            if elapsed > self.duration:
                break
            
            # Get drone position
            pos = self.interface.get_drone_position()
            if pos is None:
                rate.sleep()
                continue
            
            x, y, yaw = pos
            
            # Get camera frame
            frame = self.interface.get_camera_frame()
            if frame is not None:
                self.total_frames += 1
                
                # Detect markers
                marker_ids, marker_corners = self.interface.detect_markers()
                
                # Decode detections
                detected_cells = []
                for marker_id in marker_ids:
                    decoded = self.marker_placement.decode_id(marker_id)
                    if decoded is not None:
                        r, c, quadrant = decoded
                        detected_cells.append((r, c, quadrant))
                        self.detection_count += 1
                
                # Get most common detected cell
                detected_cell = None
                if detected_cells:
                    r, c, quadrant = detected_cells[0]
                    detected_cell = (r, c)
                
                # Create a simple drone object for controller
                from drone import Drone
                drone = Drone(x=x, y=y, yaw=yaw, z=1.0, radius=0.1)
                
                # Update controller
                self.controller.update(drone, detected_cell)
                
                # Apply velocity commands
                self.interface.set_velocity(drone.v, drone.omega)
                
                # Log
                current_cell = self.maze.world_to_cell(x, y)
                log_entry = {
                    't': elapsed,
                    'x': x,
                    'y': y,
                    'yaw': yaw,
                    'cell': current_cell,
                    'detected_markers': marker_ids,
                    'detected_cells': detected_cells
                }
                self.log.append(log_entry)
                
                # Print status
                if len(self.log) % 30 == 0:  # Every second at 30 Hz
                    detection_rate = (self.detection_count / self.total_frames * 100) if self.total_frames > 0 else 0
                    print(f"t={elapsed:.2f}s: pos=({x:.2f}, {y:.2f}), cell={current_cell}, "
                          f"detected={len(marker_ids)} markers, rate={detection_rate:.1f}%")
            
            rate.sleep()
        
        print(f"\nSimulation complete.")
        if self.total_frames > 0:
            detection_rate = self.detection_count / self.total_frames * 100
            print(f"Detection rate: {detection_rate:.1f}% ({self.detection_count}/{self.total_frames} frames)")
        
        self.interface.shutdown()


def main():
    """Main CLI function."""
    parser = argparse.ArgumentParser(
        description='Gazebo-based drone maze navigation simulation',
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
    
    args = parser.parse_args()
    
    # Validate inputs
    if args.rows < 1 or args.cols < 1:
        print("Error: rows and cols must be positive", file=sys.stderr)
        sys.exit(1)
    
    if not ROS_AVAILABLE:
        print("Error: ROS is required for Gazebo simulation.", file=sys.stderr)
        print("Please install ROS and required packages. See GAZEBO_SETUP.md", file=sys.stderr)
        sys.exit(1)
    
    try:
        sim = GazeboSimulation(
            rows=args.rows,
            cols=args.cols,
            cell_size=args.cellSize,
            marker_size=args.markerSize,
            col_radius=args.colRadius,
            wall_thickness=args.wallThickness,
            drone_radius=args.droneRadius,
            z=args.z,
            seed=args.seed,
            duration=args.seconds
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
