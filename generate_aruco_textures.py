#!/usr/bin/env python3
"""
Generate ArUco marker images as textures for Gazebo.
"""

import cv2
import numpy as np
import os
from markers import MarkerPlacement
from maze import Maze


def generate_aruco_textures(
    maze: Maze,
    marker_placement: MarkerPlacement,
    output_dir: str = "gazebo/models/aruco_marker/textures",
    size_pixels: int = 512
) -> None:
    """
    Generate ArUco marker texture images.
    
    Args:
        maze: The maze
        marker_placement: Marker placement
        output_dir: Output directory for textures
        size_pixels: Size of texture image in pixels
    """
    os.makedirs(output_dir, exist_ok=True)
    
    # Generate texture for each unique marker ID
    generated_ids = set()
    
    for marker_id, (mx, my, myaw, quadrant) in marker_placement.markers.items():
        if marker_id in generated_ids:
            continue
        
        # Generate ArUco marker image
        marker_img = marker_placement.get_marker_image(marker_id, size_pixels=size_pixels)
        
        # Resize to texture size
        marker_img = cv2.resize(marker_img, (size_pixels, size_pixels), interpolation=cv2.INTER_NEAREST)
        
        # Add border (white background)
        border_size = size_pixels // 8
        bordered = cv2.copyMakeBorder(
            marker_img,
            border_size, border_size, border_size, border_size,
            cv2.BORDER_CONSTANT,
            value=255
        )
        
        # Save as PNG
        texture_file = os.path.join(output_dir, f"marker_{marker_id}.png")
        cv2.imwrite(texture_file, bordered)
        generated_ids.add(marker_id)
    
    print(f"Generated {len(generated_ids)} ArUco marker textures in {output_dir}")


def update_aruco_model_with_texture(marker_id: int, texture_path: str) -> str:
    """
    Generate SDF for an ArUco marker with texture.
    
    Args:
        marker_id: Marker ID
        texture_path: Path to texture image
    
    Returns:
        SDF string for the marker model
    """
    sdf = f'''<?xml version="1.0"?>
<sdf version="1.6">
  <model name="aruco_marker_{marker_id}">
    <static>true</static>
    <link name="link">
      <collision name="collision">
        <geometry>
          <box>
            <size>0.1 0.1 0.001</size>
          </box>
        </geometry>
      </collision>
      <visual name="visual">
        <geometry>
          <box>
            <size>0.1 0.1 0.001</size>
          </box>
        </geometry>
        <material>
          <script>
            <uri>file://{texture_path}</uri>
            <name>MarkerTexture</name>
          </script>
          <ambient>1 1 1 1</ambient>
          <diffuse>1 1 1 1</diffuse>
        </material>
      </visual>
    </link>
  </model>
</sdf>'''
    return sdf


def main():
    """Generate ArUco textures."""
    import argparse
    
    parser = argparse.ArgumentParser(description='Generate ArUco marker textures')
    parser.add_argument('--rows', type=int, default=5, help='Maze rows')
    parser.add_argument('--cols', type=int, default=5, help='Maze columns')
    parser.add_argument('--markerSize', type=float, default=0.1, help='Marker size')
    parser.add_argument('--seed', type=int, default=None, help='Random seed')
    parser.add_argument('--textureSize', type=int, default=512, help='Texture size in pixels')
    
    args = parser.parse_args()
    
    # Create maze and markers
    maze = Maze(args.rows, args.cols, seed=args.seed)
    marker_placement = MarkerPlacement(maze, marker_size=args.markerSize)
    
    # Generate textures
    generate_aruco_textures(maze, marker_placement, size_pixels=args.textureSize)
    
    print("\nTo use these textures, you'll need to update the world generator")
    print("to reference the texture files for each marker.")


if __name__ == '__main__':
    main()
