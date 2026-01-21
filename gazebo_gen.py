#!/usr/bin/env python3
"""
Generate Gazebo world file from maze.
"""

import argparse
import sys
import cv2
import os
from maze import Maze
from markers import MarkerPlacement


def generate_gazebo_world(
    maze: Maze,
    marker_placement: MarkerPlacement,
    output_file: str = "gazebo/worlds/generated_maze.world",
    use_ros_plugins: bool = True
) -> None:
    """
    Generate a Gazebo world file with the maze.
    
    Args:
        maze: The maze
        marker_placement: Marker placement
        output_file: Output world file path
    """
    # Get absolute path for models
    import os
    # Script is in project root, so models are in gazebo/models relative to script
    script_dir = os.path.dirname(os.path.abspath(__file__))
    models_dir = os.path.join(script_dir, 'gazebo', 'models')
    
    # Verify models directory exists
    if not os.path.exists(models_dir):
        raise FileNotFoundError(f"Models directory not found: {models_dir}. Expected: {models_dir}")
    
    lines = [
        '<?xml version="1.0" ?>',
        '<sdf version="1.10">',
        '  <world name="maze_world">',
        '    ',
        '    <!-- Global light source (Gazebo Jazzy uses different format) -->',
        '    <light type="directional" name="sun">',
        '      <pose>0 0 10 0 0 0</pose>',
        '      <diffuse>0.8 0.8 0.8 1</diffuse>',
        '      <specular>0.2 0.2 0.2 1</specular>',
        '      <direction>-0.5 0.1 -0.9</direction>',
        '      <cast_shadows>true</cast_shadows>',
        '    </light>',
        '    ',
        '    <!-- Ground plane -->',
        '    <model name="ground_plane">',
        '      <static>true</static>',
        '      <link name="link">',
        '        <collision name="collision">',
        '          <geometry>',
        '            <plane>',
        '              <normal>0 0 1</normal>',
        '              <size>100 100</size>',
        '            </plane>',
        '          </geometry>',
        '        </collision>',
        '        <visual name="visual">',
        '          <geometry>',
        '            <plane>',
        '              <normal>0 0 1</normal>',
        '              <size>100 100</size>',
        '            </plane>',
        '          </geometry>',
        '          <material>',
        '            <script>',
        '              <uri>file://media/materials/scripts/gazebo.material</uri>',
        '              <name>Gazebo/White</name>',
        '            </script>',
        '          </material>',
        '        </visual>',
        '      </link>',
        '    </model>',
        '    '
    ]
    
    # Add columns at all grid vertices
    col_id = 0
    for r in range(maze.rows + 1):
        for c in range(maze.cols + 1):
            x = c * maze.cell_size
            y = r * maze.cell_size
            # Use absolute path for model
            column_model_path = os.path.join(models_dir, 'column', 'model.sdf')
            lines.append(f'    <!-- Column at ({r}, {c}) -->')
            lines.append(f'    <include>')
            lines.append(f'      <name>column_{col_id}</name>')
            lines.append(f'      <pose>{x} {y} 0.25 0 0 0</pose>')
            lines.append(f'      <uri>file://{column_model_path}</uri>')
            lines.append(f'    </include>')
            lines.append('    ')
            col_id += 1
    
    # Add wall segments
    wall_id = 0
    cell_size = maze.cell_size
    wall_thickness = maze.wall_thickness
    
    # Horizontal walls
    for r in range(maze.rows + 1):
        for c in range(maze.cols):
            has_wall = False
            if r == 0:
                has_wall = True  # Top boundary
            elif r == maze.rows:
                has_wall = True  # Bottom boundary
            else:
                has_wall = maze.has_wall(r - 1, c, 'S')
            
            if has_wall:
                x1 = c * cell_size
                y1 = r * cell_size
                x2 = (c + 1) * cell_size
                y2 = r * cell_size
                center_x = (x1 + x2) / 2.0
                center_y = (y1 + y2) / 2.0
                length = cell_size
                
                lines.append(f'    <!-- Horizontal wall at row {r}, col {c} -->')
                lines.append(f'    <model name="wall_h_{wall_id}">')
                lines.append(f'      <static>true</static>')
                lines.append(f'      <pose>{center_x} {center_y} 0.25 0 0 0</pose>')
                lines.append(f'      <link name="link">')
                lines.append(f'        <collision name="collision">')
                lines.append(f'          <geometry>')
                lines.append(f'            <box>')
                lines.append(f'              <size>{length} {wall_thickness} 0.5</size>')
                lines.append(f'            </box>')
                lines.append(f'          </geometry>')
                lines.append(f'        </collision>')
                lines.append(f'        <visual name="visual">')
                lines.append(f'          <geometry>')
                lines.append(f'            <box>')
                lines.append(f'              <size>{length} {wall_thickness} 0.5</size>')
                lines.append(f'            </box>')
                lines.append(f'          </geometry>')
                lines.append(f'          <material>')
                lines.append(f'            <script>')
                lines.append(f'              <uri>file://media/materials/scripts/gazebo.material</uri>')
                lines.append(f'              <name>Gazebo/White</name>')
                lines.append(f'            </script>')
                lines.append(f'          </material>')
                lines.append(f'        </visual>')
                lines.append(f'      </link>')
                lines.append(f'    </model>')
                lines.append('    ')
                wall_id += 1
    
    # Vertical walls
    for c in range(maze.cols + 1):
        for r in range(maze.rows):
            has_wall = False
            if c == 0:
                has_wall = True  # Left boundary
            elif c == maze.cols:
                has_wall = True  # Right boundary
            else:
                has_wall = maze.has_wall(r, c - 1, 'E')
            
            if has_wall:
                x1 = c * cell_size
                y1 = r * cell_size
                x2 = c * cell_size
                y2 = (r + 1) * cell_size
                center_x = (x1 + x2) / 2.0
                center_y = (y1 + y2) / 2.0
                length = cell_size
                
                lines.append(f'    <!-- Vertical wall at row {r}, col {c} -->')
                lines.append(f'    <model name="wall_v_{wall_id}">')
                lines.append(f'      <static>true</static>')
                lines.append(f'      <pose>{center_x} {center_y} 0.25 0 0 1.5708</pose>')
                lines.append(f'      <link name="link">')
                lines.append(f'        <collision name="collision">')
                lines.append(f'          <geometry>')
                lines.append(f'            <box>')
                lines.append(f'              <size>{length} {wall_thickness} 0.5</size>')
                lines.append(f'            </box>')
                lines.append(f'          </geometry>')
                lines.append(f'        </collision>')
                lines.append(f'        <visual name="visual">')
                lines.append(f'          <geometry>')
                lines.append(f'            <box>')
                lines.append(f'              <size>{length} {wall_thickness} 0.5</size>')
                lines.append(f'            </box>')
                lines.append(f'          </geometry>')
                lines.append(f'          <material>')
                lines.append(f'            <script>')
                lines.append(f'              <uri>file://media/materials/scripts/gazebo.material</uri>')
                lines.append(f'              <name>Gazebo/White</name>')
                lines.append(f'            </script>')
                lines.append(f'          </material>')
                lines.append(f'        </visual>')
                lines.append(f'      </link>')
                lines.append(f'    </model>')
                lines.append('    ')
                wall_id += 1
    
    # Add ArUco markers with actual ArUco patterns as textures
    marker_id = 0
    textures_dir = os.path.join(models_dir, 'aruco_marker', 'textures')
    os.makedirs(textures_dir, exist_ok=True)
    
    # Generate marker textures first
    generated_textures = {}
    for marker_id_val, (mx, my, myaw, quadrant) in marker_placement.markers.items():
        if marker_id_val not in generated_textures:
            # Generate ArUco marker image
            marker_img = marker_placement.get_marker_image(marker_id_val, size_pixels=512)
            # Resize and add white border
            marker_img = cv2.resize(marker_img, (512, 512), interpolation=cv2.INTER_NEAREST)
            border_size = 64
            bordered = cv2.copyMakeBorder(marker_img, border_size, border_size, border_size, border_size, cv2.BORDER_CONSTANT, value=255)
            # Save texture
            texture_file = os.path.join(textures_dir, f"marker_{marker_id_val}.png")
            cv2.imwrite(texture_file, bordered)
            generated_textures[marker_id_val] = texture_file
    
    # Now add markers to world with textures
    for marker_id_val, (mx, my, myaw, quadrant) in marker_placement.markers.items():
        # Create a simple visual representation
        # In a real setup, you'd use an ArUco marker plugin or texture
        lines.append(f'    <!-- ArUco marker ID {marker_id_val} at ({mx:.3f}, {my:.3f}) -->')
        lines.append(f'    <model name="aruco_{marker_id}">')
        lines.append(f'      <static>true</static>')
        lines.append(f'      <pose>{mx} {my} 0.0005 0 0 {myaw}</pose>')
        lines.append(f'      <link name="link">')
        texture_path = generated_textures[marker_id_val]
        abs_texture_path = os.path.abspath(texture_path)
        lines.append(f'        <collision name="collision">')
        lines.append(f'          <geometry>')
        lines.append(f'            <box>')
        lines.append(f'              <size>{marker_placement.marker_size} {marker_placement.marker_size} 0.001</size>')
        lines.append(f'            </box>')
        lines.append(f'          </geometry>')
        lines.append(f'        </collision>')
        lines.append(f'        <visual name="visual">')
        lines.append(f'          <geometry>')
        lines.append(f'            <box>')
        lines.append(f'              <size>{marker_placement.marker_size} {marker_placement.marker_size} 0.001</size>')
        lines.append(f'            </box>')
        lines.append(f'          </geometry>')
        lines.append(f'          <material>')
        lines.append(f'            <pbr>')
        lines.append(f'              <metal>')
        lines.append(f'                <albedo_map>file://{abs_texture_path}</albedo_map>')
        lines.append(f'                <roughness>1.0</roughness>')
        lines.append(f'                <metalness>0.0</metalness>')
        lines.append(f'              </metal>')
        lines.append(f'            </pbr>')
        lines.append(f'          </material>')
        lines.append(f'        </visual>')
        lines.append(f'      </link>')
        lines.append(f'    </model>')
        lines.append('    ')
        marker_id += 1
    
    # Add drone (optionally without ROS plugins)
    start_x, start_y = maze.cell_to_world_center(0, 0)
    drone_model_file = 'model.sdf' if use_ros_plugins else 'model_no_ros.sdf'
    drone_model_path = os.path.join(models_dir, 'drone', drone_model_file)
    lines.append('    <!-- Drone -->')
    lines.append('    <include>')
    lines.append(f'      <name>drone</name>')
    lines.append(f'      <pose>{start_x} {start_y} 1.0 0 0 0</pose>')
    lines.append(f'      <uri>file://{drone_model_path}</uri>')
    lines.append('    </include>')
    lines.append('    ')
    
    lines.append('  </world>')
    lines.append('</sdf>')
    
    # Write to file
    with open(output_file, 'w') as f:
        f.write('\n'.join(lines))
    
    print(f"Generated Gazebo world file: {output_file}")
    print(f"  - {col_id} columns")
    print(f"  - {wall_id} wall segments")
    print(f"  - {marker_id} ArUco markers")


def main():
    """CLI for generating Gazebo world."""
    parser = argparse.ArgumentParser(description='Generate Gazebo world from maze')
    parser.add_argument('--rows', type=int, default=5, help='Maze rows')
    parser.add_argument('--cols', type=int, default=5, help='Maze columns')
    parser.add_argument('--cellSize', type=float, default=1.0, help='Cell size in meters')
    parser.add_argument('--markerSize', type=float, default=0.1, help='Marker size in meters')
    parser.add_argument('--colRadius', type=float, default=0.05, help='Column radius')
    parser.add_argument('--wallThickness', type=float, default=0.02, help='Wall thickness')
    parser.add_argument('--seed', type=int, default=None, help='Random seed')
    parser.add_argument('--out', type=str, default='gazebo/worlds/generated_maze.world', help='Output file')
    parser.add_argument('--no-ros', action='store_true', help='Use drone model without ROS plugins')
    
    args = parser.parse_args()
    
    # Generate maze
    maze = Maze(
        args.rows, args.cols,
        cell_size=args.cellSize,
        wall_thickness=args.wallThickness,
        col_radius=args.colRadius,
        seed=args.seed
    )
    
    # Generate markers
    marker_placement = MarkerPlacement(maze, marker_size=args.markerSize)
    
    # Generate Gazebo world
    generate_gazebo_world(maze, marker_placement, args.out, use_ros_plugins=not args.no_ros)


if __name__ == '__main__':
    main()
