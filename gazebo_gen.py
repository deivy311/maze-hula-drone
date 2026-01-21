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


def direction_to_yaw(direction: str) -> float:
    """Convert direction string to yaw angle in radians."""
    direction_map = {
        'North': 0.0,
        'East': 1.5708,  # 90 degrees
        'South': 3.14159,  # 180 degrees
        'West': -1.5708  # -90 degrees or 4.71239
    }
    return direction_map.get(direction, 0.0)


def generate_gazebo_world(
    maze: Maze,
    marker_placement: MarkerPlacement,
    output_file: str = "gazebo/worlds/generated_maze.world",
    use_ros_plugins: bool = True,
    objects: list = None,
    phase: str = "race"  # "discovery" or "race"
) -> None:
    """
    Generate a Gazebo world file with the maze.
    
    Args:
        maze: The maze
        marker_placement: Marker placement
        output_file: Output world file path
        use_ros_plugins: Whether to use ROS plugins
        objects: List of objects, each as dict with keys: 'x', 'y', 'direction', 'type'
                 where direction is 'North', 'East', 'South', or 'West'
                 and type is 'house', 'tank', or 'tree'
        phase: "discovery" (no objects) or "race" (with objects)
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
    
    # Add columns at cell corners (where 4 ArUcos from 4 adjacent cells meet)
    # Each cell has 2x2 ArUcos, so columns are placed every 2 ArUcos in each direction
    # Columns are positioned at the MIDDLE between the corners of adjacent ArUcos
    # Calculate marker spacing (same as in markers.py)
    marker_size = marker_placement.marker_size
    gap = marker_placement.gap
    marker_spacing = marker_size + gap
    start_offset = marker_size / 2.0 + gap
    
    col_id = 0
    # Columns are at cell corners: middle between corners of 4 ArUcos
    # For cell (r, c), its corners are:
    # - Top-left corner: middle between top-left corner of ArUco (2*r, 2*c) and top-right corner of ArUco (2*r, 2*c-1)
    #   and between top-left corner of ArUco (2*r, 2*c) and bottom-left corner of ArUco (2*r-1, 2*c)
    #
    # Actually simpler: for cell (r, c) with ArUcos at positions:
    # - Top-left ArUco at: (start_offset + 2*c * marker_spacing, start_offset + 2*r * marker_spacing)
    #   Its corners: top-left at (x - marker_size/2, y - marker_size/2), top-right at (x + marker_size/2, y - marker_size/2)
    # - Top-right ArUco at: (start_offset + (2*c+1) * marker_spacing, start_offset + 2*r * marker_spacing)
    #   Its corners: top-left at (x - marker_size/2, y - marker_size/2), top-right at (x + marker_size/2, y - marker_size/2)
    #
    # Column at top-left corner of cell (r, c) is at the middle between:
    # - Top-right corner of top-left ArUco: (start_offset + 2*c * marker_spacing + marker_size/2, start_offset + 2*r * marker_spacing - marker_size/2)
    # - Top-left corner of top-right ArUco: (start_offset + (2*c+1) * marker_spacing - marker_size/2, start_offset + 2*r * marker_spacing - marker_size/2)
    # Middle X: (start_offset + 2*c * marker_spacing + marker_size/2 + start_offset + (2*c+1) * marker_spacing - marker_size/2) / 2
    #         = (2*start_offset + 2*c * marker_spacing + marker_size/2 + (2*c+1) * marker_spacing - marker_size/2) / 2
    #         = (2*start_offset + 2*c * marker_spacing + 2*c * marker_spacing + marker_spacing) / 2
    #         = (2*start_offset + 4*c * marker_spacing + marker_spacing) / 2
    #         = start_offset + 2*c * marker_spacing + marker_spacing/2
    
    # Place columns at all cell corners (every 2 ArUcos)
    for r in range(maze.rows + 1):
        for c in range(maze.cols + 1):
            # Column at corner: middle between corners of adjacent ArUcos
            # Top-left corner of cell (r, c): middle between top-right corner of ArUco (2*r, 2*c-1) and top-left corner of ArUco (2*r, 2*c)
            # But simpler: it's at the intersection where 4 ArUcos meet
            # For cell corner, we need the middle between:
            # - Horizontally: between right edge of left ArUco and left edge of right ArUco
            # - Vertically: between bottom edge of top ArUco and top edge of bottom ArUco
            
            # Top-left corner of cell (r, c):
            # - Left ArUco center: start_offset + (2*c-1) * marker_spacing (if c > 0)
            # - Right ArUco center: start_offset + 2*c * marker_spacing
            # Middle X: start_offset + ((2*c-1) + 2*c) * marker_spacing / 2 = start_offset + (4*c - 1) * marker_spacing / 2
            # But for c=0, we use the left boundary
            
            # Actually, let's think of it as the middle between the corners of the 2x2 ArUco block
            # For cell (r, c), the 4 ArUcos are at grid positions (2*r, 2*c), (2*r, 2*c+1), (2*r+1, 2*c), (2*r+1, 2*c+1)
            # Top-left corner is where these 4 ArUcos meet
            # It's the middle between:
            # - Top-right corner of (2*r, 2*c-1) and top-left corner of (2*r, 2*c) horizontally
            # - Bottom-left corner of (2*r-1, 2*c) and top-left corner of (2*r, 2*c) vertically
            
            # For the top-left corner of cell (r, c):
            # If c > 0: middle between ArUco (2*r, 2*c-1) right edge and ArUco (2*r, 2*c) left edge
            #   x = (start_offset + (2*c-1) * marker_spacing + marker_size/2 + start_offset + 2*c * marker_spacing - marker_size/2) / 2
            #     = start_offset + (2*c-1 + 2*c) * marker_spacing / 2 = start_offset + (4*c - 1) * marker_spacing / 2
            # If c == 0: at the left boundary, x = start_offset - marker_size/2 - gap/2
            # Similar for y
            
            # Column at corner: middle between corners of adjacent ArUcos
            # For cell corner (r, c), we need the middle between:
            # - Horizontally: right corner of left ArUco and left corner of right ArUco
            # - Vertically: bottom corner of top ArUco and top corner of bottom ArUco
            
            if c == 0:
                # Left boundary: at the left edge
                x = start_offset - marker_size / 2.0 - gap / 2.0
            else:
                # Middle between right corner of ArUco at (2*r, 2*c-1) and left corner of ArUco at (2*r, 2*c)
                # Right corner of left ArUco: center + marker_size/2
                left_aruco_center = start_offset + (2 * c - 1) * marker_spacing
                right_corner_x = left_aruco_center + marker_size / 2.0
                # Left corner of right ArUco: center - marker_size/2
                right_aruco_center = start_offset + 2 * c * marker_spacing
                left_corner_x = right_aruco_center - marker_size / 2.0
                # Middle point
                x = (right_corner_x + left_corner_x) / 2.0
            
            if r == 0:
                # Top boundary: at the top edge
                y = start_offset - marker_size / 2.0 - gap / 2.0
            else:
                # Middle between bottom corner of ArUco at (2*r-1, 2*c) and top corner of ArUco at (2*r, 2*c)
                # Bottom corner of top ArUco: center + marker_size/2
                top_aruco_center = start_offset + (2 * r - 1) * marker_spacing
                bottom_corner_y = top_aruco_center + marker_size / 2.0
                # Top corner of bottom ArUco: center - marker_size/2
                bottom_aruco_center = start_offset + 2 * r * marker_spacing
                top_corner_y = bottom_aruco_center - marker_size / 2.0
                # Middle point
                y = (bottom_corner_y + top_corner_y) / 2.0
            column_model_path = os.path.join(models_dir, 'column', 'model.sdf')
            # Columns: from floor (0.0m) to half the previous height
            column_bottom = 0.0
            column_height = 0.9  # Half of 1.8m
            column_top = column_bottom + column_height  # 0.9m
            column_center_z = (column_bottom + column_top) / 2.0  # 0.45m
            
            lines.append(f'    <!-- Column at cell corner ({r}, {c}) -->')
            lines.append(f'    <include>')
            lines.append(f'      <name>column_{col_id}</name>')
            lines.append(f'      <pose>{x} {y} {column_center_z} 0 0 0</pose>')
            lines.append(f'      <uri>file://{column_model_path}</uri>')
            lines.append(f'    </include>')
            lines.append('    ')
            col_id += 1
    
    # Add wall segments
    # Walls are placed in the middle between adjacent markers (same position as columns)
    wall_id = 0
    # Use thinner walls (0.005)
    wall_thickness = min(maze.wall_thickness, 0.005)
    
    # Calculate marker spacing and start position (same as in markers.py)
    marker_size = marker_placement.marker_size
    gap = marker_placement.gap
    marker_spacing = marker_size + gap  # 1.5 * marker_size
    start_offset = marker_size / 2.0 + gap
    
    # Calculate cell size based on marker spacing (2 markers per cell)
    cell_size = 2 * marker_spacing  # 3 * marker_size
    
    # Horizontal walls (between ArUcos in same column, where there are walls)
    # Walls are placed in the middle between adjacent ArUcos, same position as columns
    total_marker_rows = 2 * maze.rows
    total_marker_cols = 2 * maze.cols
    
    # Horizontal walls: connect columns horizontally where there are walls
    # Walls extend from one column to the next column (every 2 ArUcos)
    # Check walls between cell rows
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
                # Wall extends horizontally from column at (r, c) to column at (r, c+1)
                # Column positions (same calculation as above)
                if c == 0:
                    x_start = start_offset - marker_size / 2.0 - gap / 2.0
                else:
                    left_aruco_center = start_offset + (2 * c - 1) * marker_spacing
                    right_corner_x = left_aruco_center + marker_size / 2.0
                    right_aruco_center = start_offset + 2 * c * marker_spacing
                    left_corner_x = right_aruco_center - marker_size / 2.0
                    x_start = (right_corner_x + left_corner_x) / 2.0
                
                # Column at (r, c+1)
                left_aruco_center = start_offset + (2 * (c + 1) - 1) * marker_spacing
                right_corner_x = left_aruco_center + marker_size / 2.0
                right_aruco_center = start_offset + 2 * (c + 1) * marker_spacing
                left_corner_x = right_aruco_center - marker_size / 2.0
                x_end = (right_corner_x + left_corner_x) / 2.0
                
                # Wall Y position: same as column Y position for row r
                if r == 0:
                    y = start_offset - marker_size / 2.0 - gap / 2.0
                else:
                    top_aruco_center = start_offset + (2 * r - 1) * marker_spacing
                    bottom_corner_y = top_aruco_center + marker_size / 2.0
                    bottom_aruco_center = start_offset + 2 * r * marker_spacing
                    top_corner_y = bottom_aruco_center - marker_size / 2.0
                    y = (bottom_corner_y + top_corner_y) / 2.0
                
                center_x = (x_start + x_end) / 2.0
                center_y = y
                length = x_end - x_start
                
                # Wall height: from 0.3m (3x the previous 0.1m gap) to half the previous height
                floor_gap = 0.3  # 3x the previous 0.1m
                wall_bottom = floor_gap
                wall_height = 0.75  # Half of 1.5m
                wall_top = wall_bottom + wall_height  # 1.05m
                wall_center_z = (wall_bottom + wall_top) / 2.0  # 0.675m
                
                lines.append(f'    <!-- Horizontal wall between columns at row {r}, cols {c} and {c+1} -->')
                lines.append(f'    <model name="wall_h_{wall_id}">')
                lines.append(f'      <static>true</static>')
                lines.append(f'      <pose>{center_x} {center_y} {wall_center_z} 0 0 0</pose>')
                lines.append(f'      <link name="link">')
                lines.append(f'        <collision name="collision">')
                lines.append(f'          <geometry>')
                lines.append(f'            <box>')
                lines.append(f'              <size>{length} {wall_thickness} {wall_height}</size>')
                lines.append(f'            </box>')
                lines.append(f'          </geometry>')
                lines.append(f'        </collision>')
                lines.append(f'        <visual name="visual">')
                lines.append(f'          <geometry>')
                lines.append(f'            <box>')
                lines.append(f'              <size>{length} {wall_thickness} {wall_height}</size>')
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
    
    # Vertical walls: connect columns vertically where there are walls
    # Walls extend from one column to the next column (every 2 ArUcos)
    # Check walls between cell columns
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
                # Wall extends vertically from column at (r, c) to column at (r+1, c)
                # Column positions (same calculation as above)
                if r == 0:
                    y_start = start_offset - marker_size / 2.0 - gap / 2.0
                else:
                    top_aruco_center = start_offset + (2 * r - 1) * marker_spacing
                    bottom_corner_y = top_aruco_center + marker_size / 2.0
                    bottom_aruco_center = start_offset + 2 * r * marker_spacing
                    top_corner_y = bottom_aruco_center - marker_size / 2.0
                    y_start = (bottom_corner_y + top_corner_y) / 2.0
                
                # Column at (r+1, c)
                top_aruco_center = start_offset + (2 * (r + 1) - 1) * marker_spacing
                bottom_corner_y = top_aruco_center + marker_size / 2.0
                bottom_aruco_center = start_offset + 2 * (r + 1) * marker_spacing
                top_corner_y = bottom_aruco_center - marker_size / 2.0
                y_end = (bottom_corner_y + top_corner_y) / 2.0
                
                # Wall X position: same as column X position for col c
                if c == 0:
                    x = start_offset - marker_size / 2.0 - gap / 2.0
                else:
                    left_aruco_center = start_offset + (2 * c - 1) * marker_spacing
                    right_corner_x = left_aruco_center + marker_size / 2.0
                    right_aruco_center = start_offset + 2 * c * marker_spacing
                    left_corner_x = right_aruco_center - marker_size / 2.0
                    x = (right_corner_x + left_corner_x) / 2.0
                
                center_x = x
                center_y = (y_start + y_end) / 2.0
                length = y_end - y_start
                
                # Wall height: from 0.3m (3x the previous 0.1m gap) to half the previous height
                floor_gap = 0.3  # 3x the previous 0.1m
                wall_bottom = floor_gap
                wall_height = 0.75  # Half of 1.5m
                wall_top = wall_bottom + wall_height  # 1.05m
                wall_center_z = (wall_bottom + wall_top) / 2.0  # 0.675m
                
                lines.append(f'    <!-- Vertical wall between columns at col {c}, rows {r} and {r+1} -->')
                lines.append(f'    <model name="wall_v_{wall_id}">')
                lines.append(f'      <static>true</static>')
                lines.append(f'      <pose>{center_x} {center_y} {wall_center_z} 0 0 0</pose>')
                lines.append(f'      <link name="link">')
                lines.append(f'        <collision name="collision">')
                lines.append(f'          <geometry>')
                lines.append(f'            <box>')
                lines.append(f'              <size>{wall_thickness} {length} {wall_height}</size>')
                lines.append(f'            </box>')
                lines.append(f'          </geometry>')
                lines.append(f'        </collision>')
                lines.append(f'        <visual name="visual">')
                lines.append(f'          <geometry>')
                lines.append(f'            <box>')
                lines.append(f'              <size>{wall_thickness} {length} {wall_height}</size>')
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
            # Generate ArUco marker image (grayscale)
            marker_img = marker_placement.get_marker_image(marker_id_val, size_pixels=512)
            # Resize and add white border
            marker_img = cv2.resize(marker_img, (512, 512), interpolation=cv2.INTER_NEAREST)
            border_size = 64
            bordered = cv2.copyMakeBorder(marker_img, border_size, border_size, border_size, border_size, cv2.BORDER_CONSTANT, value=255)
            # Convert grayscale to RGB (3 channels) for better Gazebo compatibility
            if len(bordered.shape) == 2:
                bordered_rgb = cv2.cvtColor(bordered, cv2.COLOR_GRAY2RGB)
            else:
                bordered_rgb = bordered
            # Save texture as RGB PNG
            texture_file = os.path.join(textures_dir, f"marker_{marker_id_val}.png")
            cv2.imwrite(texture_file, bordered_rgb)
            generated_textures[marker_id_val] = texture_file
    
    # Now add markers to world with textures
    for marker_id_val, (mx, my, myaw, quadrant) in marker_placement.markers.items():
        # Create a simple visual representation
        # In a real setup, you'd use an ArUco marker plugin or texture
        lines.append(f'    <!-- ArUco marker ID {marker_id_val} at ({mx:.3f}, {my:.3f}) -->')
        lines.append(f'    <model name="aruco_{marker_id}">')
        lines.append(f'      <static>true</static>')
        lines.append(f'      <pose>{mx} {my} 0.001 0 0 {myaw}</pose>')
        lines.append(f'      <link name="link">')
        texture_path = generated_textures[marker_id_val]
        abs_texture_path = os.path.abspath(texture_path)
        lines.append(f'        <collision name="collision">')
        lines.append(f'          <geometry>')
        lines.append(f'            <box>')
        lines.append(f'              <size>{marker_placement.marker_size} {marker_placement.marker_size} 0.002</size>')
        lines.append(f'            </box>')
        lines.append(f'          </geometry>')
        lines.append(f'        </collision>')
        lines.append(f'        <visual name="visual">')
        lines.append(f'          <geometry>')
        lines.append(f'            <box>')
        lines.append(f'              <size>{marker_placement.marker_size} {marker_placement.marker_size} 0.002</size>')
        lines.append(f'            </box>')
        lines.append(f'          </geometry>')
        # Use model:// URI format which is the standard for Gazebo
        # Calculate relative path from models directory
        texture_rel_path = os.path.relpath(abs_texture_path, models_dir)
        texture_uri = f"model://aruco_marker/textures/{os.path.basename(abs_texture_path)}"
        
        lines.append(f'          <material>')
        lines.append(f'            <pbr>')
        lines.append(f'              <metal>')
        lines.append(f'                <albedo_map>{texture_uri}</albedo_map>')
        lines.append(f'                <roughness>0.2</roughness>')
        lines.append(f'                <metalness>0.0</metalness>')
        lines.append(f'              </metal>')
        lines.append(f'            </pbr>')
        lines.append(f'            <diffuse>1.0 1.0 1.0 1.0</diffuse>')
        lines.append(f'            <ambient>1.0 1.0 1.0 1.0</ambient>')
        lines.append(f'          </material>')
        lines.append(f'        </visual>')
        lines.append(f'      </link>')
        lines.append(f'    </model>')
        lines.append('    ')
        marker_id += 1
    
    # Add objects as images on walls (only in race phase)
    if objects is None:
        objects = []
    
    if phase == "race" and objects:
        object_height = 0.90  # Objects at 90cm height as per challenge rules
        object_size = 0.3  # Size of object image (30cm)
        object_id = 0
        textures_dir = os.path.join(models_dir, 'object_images', 'textures')
        
        # Use same coordinate system as ArUcos and walls
        marker_size = marker_placement.marker_size
        gap = marker_placement.gap
        marker_spacing = marker_size + gap
        start_offset = marker_size / 2.0 + gap
        
        for obj in objects:
            obj_x = obj['x']  # Column index
            obj_y = obj['y']  # Row index
            obj_direction = obj.get('direction', 'North')
            obj_type = obj.get('type', 'house')  # house, tank, or tree
            
            # Get image path (prefer PNG, fallback to JPG)
            image_file_png = f"{obj_type}.png"
            image_file_jpg = f"{obj_type}.jpg"
            image_path_png = os.path.join(textures_dir, image_file_png)
            image_path_jpg = os.path.join(textures_dir, image_file_jpg)
            
            if os.path.exists(image_path_png):
                abs_image_path = os.path.abspath(image_path_png)
            elif os.path.exists(image_path_jpg):
                abs_image_path = os.path.abspath(image_path_jpg)
            else:
                print(f"Warning: Image not found for {obj_type} at {textures_dir}")
                continue
            
            # Calculate cell center using same system as ArUcos
            # Cell (r, c) center is at: start_offset + marker_spacing*(c + 0.5), start_offset + marker_spacing*(r + 0.5)
            cell_center_x = start_offset + marker_spacing * (obj_x + 0.5)
            cell_center_y = start_offset + marker_spacing * (obj_y + 0.5)
            
            # Calculate wall position using same logic as columns/walls
            # Walls are at the same positions as columns (middle between ArUcos)
            # For North wall: between row r and r+1 of ArUcos
            # For East wall: between col c and c+1 of ArUcos
            if obj_direction == 'North':
                # North wall is at the boundary between this cell and the cell above
                # Wall Y position: same as column at row r+1 (if exists) or boundary
                if obj_y == maze.rows - 1:
                    # Top boundary
                    wall_y = start_offset - marker_size / 2.0 - gap / 2.0 + (2 * (obj_y + 1)) * marker_spacing
                else:
                    # Middle between ArUcos at row 2*r+1 and 2*r+2
                    top_aruco_center = start_offset + (2 * obj_y + 1) * marker_spacing
                    bottom_corner_y = top_aruco_center + marker_size / 2.0
                    bottom_aruco_center = start_offset + (2 * (obj_y + 1)) * marker_spacing
                    top_corner_y = bottom_aruco_center - marker_size / 2.0
                    wall_y = (bottom_corner_y + top_corner_y) / 2.0
                wall_x = cell_center_x
                wall_roll = 0.0
                wall_pitch = 1.5708  # 90 degrees - rotate to stand vertical
                wall_yaw = 0.0  # Facing North
            elif obj_direction == 'East':
                # East wall is at the boundary between this cell and the cell to the right
                if obj_x == maze.cols - 1:
                    # Right boundary
                    wall_x = start_offset - marker_size / 2.0 - gap / 2.0 + (2 * (obj_x + 1)) * marker_spacing
                else:
                    # Middle between ArUcos at col 2*c+1 and 2*c+2
                    left_aruco_center = start_offset + (2 * obj_x + 1) * marker_spacing
                    right_corner_x = left_aruco_center + marker_size / 2.0
                    right_aruco_center = start_offset + (2 * (obj_x + 1)) * marker_spacing
                    left_corner_x = right_aruco_center - marker_size / 2.0
                    wall_x = (right_corner_x + left_corner_x) / 2.0
                wall_y = cell_center_y
                wall_roll = 0.0
                wall_pitch = 1.5708  # 90 degrees - rotate to stand vertical
                wall_yaw = 1.5708  # 90 degrees - facing East
            elif obj_direction == 'South':
                # South wall is at the boundary between this cell and the cell below
                if obj_y == 0:
                    # Bottom boundary
                    wall_y = start_offset - marker_size / 2.0 - gap / 2.0
                else:
                    # Middle between ArUcos at row 2*r-1 and 2*r
                    top_aruco_center = start_offset + (2 * obj_y - 1) * marker_spacing
                    bottom_corner_y = top_aruco_center + marker_size / 2.0
                    bottom_aruco_center = start_offset + 2 * obj_y * marker_spacing
                    top_corner_y = bottom_aruco_center - marker_size / 2.0
                    wall_y = (bottom_corner_y + top_corner_y) / 2.0
                wall_x = cell_center_x
                wall_roll = 0.0
                wall_pitch = 1.5708  # 90 degrees - rotate to stand vertical
                wall_yaw = 3.14159  # 180 degrees - facing South
            else:  # West
                # West wall is at the boundary between this cell and the cell to the left
                if obj_x == 0:
                    # Left boundary
                    wall_x = start_offset - marker_size / 2.0 - gap / 2.0
                else:
                    # Middle between ArUcos at col 2*c-1 and 2*c
                    left_aruco_center = start_offset + (2 * obj_x - 1) * marker_spacing
                    right_corner_x = left_aruco_center + marker_size / 2.0
                    right_aruco_center = start_offset + 2 * obj_x * marker_spacing
                    left_corner_x = right_aruco_center - marker_size / 2.0
                    wall_x = (right_corner_x + left_corner_x) / 2.0
                wall_y = cell_center_y
                wall_roll = 0.0
                wall_pitch = 1.5708  # 90 degrees - rotate to stand vertical
                wall_yaw = -1.5708  # -90 degrees - facing West
            
            # Create a thin box (vertical plane) with the image as texture
            # Box size: width (X) x height (Y) x depth (Z)
            # After pitch=90 rotation, the box stands vertical
            # Position the image on the wall surface (slightly offset to avoid z-fighting)
            
            # Offset from wall surface: half wall thickness + half box depth + small gap
            wall_thickness_half = 0.0025  # Half of 0.005m wall thickness
            box_depth_half = 0.0005  # Half of 0.001m box depth
            gap = 0.001  # Small gap to avoid z-fighting
            offset_distance = wall_thickness_half + box_depth_half + gap
            
            if obj_direction == 'North':
                # Move North (positive Y) from wall center to surface
                image_x = wall_x
                image_y = wall_y + offset_distance
            elif obj_direction == 'East':
                # Move East (positive X) from wall center to surface
                image_x = wall_x + offset_distance
                image_y = wall_y
            elif obj_direction == 'South':
                # Move South (negative Y) from wall center to surface
                image_x = wall_x
                image_y = wall_y - offset_distance
            else:  # West
                # Move West (negative X) from wall center to surface
                image_x = wall_x - offset_distance
                image_y = wall_y
            
            lines.append(f'    <!-- Object {object_id}: {obj_type} image at cell ({obj_x}, {obj_y}), direction {obj_direction} -->')
            lines.append(f'    <model name="object_image_{object_id}">')
            lines.append(f'      <static>true</static>')
            lines.append(f'      <pose>{image_x} {image_y} {object_height} {wall_roll} {wall_pitch} {wall_yaw}</pose>')
            lines.append(f'      <link name="link">')
            lines.append(f'        <collision name="collision">')
            lines.append(f'          <geometry>')
            lines.append(f'            <box>')
            lines.append(f'              <size>{object_size} {object_size} 0.001</size>')
            lines.append(f'            </box>')
            lines.append(f'          </geometry>')
            lines.append(f'        </collision>')
            lines.append(f'        <visual name="visual">')
            lines.append(f'          <geometry>')
            lines.append(f'            <box>')
            lines.append(f'              <size>{object_size} {object_size} 0.001</size>')
            lines.append(f'            </box>')
            lines.append(f'          </geometry>')
            # Use model:// URI format which is the standard for Gazebo
            object_texture_uri = f"model://object_images/textures/{os.path.basename(abs_image_path)}"
            
            lines.append(f'          <material>')
            lines.append(f'            <pbr>')
            lines.append(f'              <metal>')
            lines.append(f'                <albedo_map>{object_texture_uri}</albedo_map>')
            lines.append(f'                <roughness>0.2</roughness>')
            lines.append(f'                <metalness>0.0</metalness>')
            lines.append(f'              </metal>')
            lines.append(f'            </pbr>')
            lines.append(f'            <diffuse>1.0 1.0 1.0 1.0</diffuse>')
            lines.append(f'            <ambient>1.0 1.0 1.0 1.0</ambient>')
            lines.append(f'          </material>')
            lines.append(f'        </visual>')
            lines.append(f'      </link>')
            lines.append(f'    </model>')
            lines.append('    ')
            object_id += 1
    
    # Add drone (optionally without ROS plugins)
    # Place drone at center of cell (0, 0)
    # Cell center is at: middle of the 4 markers of that cell
    # Markers of cell (0, 0) are at:
    #   Top-left: start_offset, start_offset
    #   Top-right: start_offset + marker_spacing, start_offset
    #   Bottom-left: start_offset, start_offset + marker_spacing
    #   Bottom-right: start_offset + marker_spacing, start_offset + marker_spacing
    # Center: start_offset + marker_spacing/2, start_offset + marker_spacing/2
    start_x = start_offset + marker_spacing / 2.0
    start_y = start_offset + marker_spacing / 2.0
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
    if phase == "race" and objects:
        print(f"  - {len(objects)} objects at 0.90m height")


def main():
    """CLI for generating Gazebo world."""
    parser = argparse.ArgumentParser(description='Generate Gazebo world from maze')
    parser.add_argument('--rows', type=int, default=5, help='Maze rows')
    parser.add_argument('--cols', type=int, default=5, help='Maze columns')
    parser.add_argument('--cellSize', type=float, default=1.0, help='Cell size in meters')
    parser.add_argument('--markerSize', type=float, default=0.1, help='Marker size in meters')
    parser.add_argument('--colRadius', type=float, default=0.01, help='Column radius (default: 0.01)')
    parser.add_argument('--wallThickness', type=float, default=0.005, help='Wall thickness (default: 0.005)')
    parser.add_argument('--seed', type=int, default=None, help='Random seed')
    parser.add_argument('--out', type=str, default='gazebo/worlds/generated_maze.world', help='Output file')
    parser.add_argument('--no-ros', action='store_true', help='Use drone model without ROS plugins')
    parser.add_argument('--phase', type=str, default='race', choices=['discovery', 'race'], 
                        help='Competition phase: discovery (no objects) or race (with objects)')
    parser.add_argument('--objects', type=str, default=None, 
                        help='JSON file with objects list, or JSON string. Format: [{"x": int, "y": int, "direction": "North|East|South|West", "type": "house|tank|tree"}]')
    
    args = parser.parse_args()
    
    # Parse objects
    objects = None
    if args.objects:
        import json
        if os.path.isfile(args.objects):
            with open(args.objects, 'r') as f:
                objects = json.load(f)
        else:
            # Try to parse as JSON string
            try:
                objects = json.loads(args.objects)
            except json.JSONDecodeError:
                print(f"Error: Could not parse objects as JSON file or string: {args.objects}")
                sys.exit(1)
    
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
    generate_gazebo_world(
        maze, 
        marker_placement, 
        args.out, 
        use_ros_plugins=not args.no_ros,
        objects=objects,
        phase=args.phase
    )


if __name__ == '__main__':
    main()
