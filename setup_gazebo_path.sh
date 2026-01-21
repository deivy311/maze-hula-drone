#!/bin/bash
# Setup script for Gazebo Jazzy model path

# Get the directory where this script is located
SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
MODELS_DIR="$SCRIPT_DIR/gazebo/models"

# Set Gazebo Jazzy resource path
export GZ_SIM_RESOURCE_PATH="$GZ_SIM_RESOURCE_PATH:$MODELS_DIR"

echo "Gazebo Jazzy model path set to: $MODELS_DIR"
echo "To make this permanent, add to your ~/.bashrc:"
echo "export GZ_SIM_RESOURCE_PATH=\"\$GZ_SIM_RESOURCE_PATH:$MODELS_DIR\""
