#!/bin/bash
# Quick script to run the simulation

echo "=== Maze Drone Simulation ==="
echo ""

# Check ROS 2 (prefer Jazzy for Gazebo Jazzy compatibility)
if [ -f /opt/ros/jazzy/setup.bash ]; then
    source /opt/ros/jazzy/setup.bash
    echo "✓ ROS 2 Jazzy loaded (compatible with Gazebo Jazzy)"
elif [ -f /opt/ros/rolling/setup.bash ]; then
    source /opt/ros/rolling/setup.bash
    echo "✓ ROS 2 Rolling loaded"
elif [ -f /opt/ros/humble/setup.bash ]; then
    source /opt/ros/humble/setup.bash
    echo "✓ ROS 2 Humble loaded"
else
    echo "⚠ ROS 2 not found. Simulation may work but ROS features won't."
fi

# Set Gazebo path
source setup_gazebo_path.sh

# Parse args
REGENERATE=0
NO_ROS=0
USE_ROS=0
for arg in "$@"; do
    case "$arg" in
        --regenerate) REGENERATE=1 ;;
        --no-ros) NO_ROS=1 ;;
        --ros) USE_ROS=1 ;;
    esac
done

# If user explicitly wants ROS plugins
if [ "$USE_ROS" -eq 1 ]; then
    NO_ROS=0
fi

# Generate world if it doesn't exist or if requested
if [ "$REGENERATE" -eq 1 ] || [ ! -f "gazebo/worlds/generated_maze.world" ]; then
    echo "Generating maze world..."
    if [ "$NO_ROS" -eq 1 ]; then
        echo "Using drone model without ROS plugins (avoids missing plugin errors)"
        python3 gazebo_gen.py --rows 5 --cols 5 --no-ros
    else
        echo "Using drone model with ROS plugins"
        python3 gazebo_gen.py --rows 5 --cols 5
    fi
fi

echo ""
echo "Launching Gazebo..."
echo "Press Ctrl+C to stop"
echo ""

gz sim gazebo/worlds/generated_maze.world
