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
PHASE="race"
ROWS=5
COLS=5
OBJECTS_FILE=""
for arg in "$@"; do
    case "$arg" in
        --regenerate) REGENERATE=1 ;;
        --no-ros) NO_ROS=1 ;;
        --ros) USE_ROS=1 ;;
        --phase=*) PHASE="${arg#*=}" ;;
        --rows=*) ROWS="${arg#*=}" ;;
        --cols=*) COLS="${arg#*=}" ;;
        --objects=*) OBJECTS_FILE="${arg#*=}" ;;
    esac
done

# If user explicitly wants ROS plugins
if [ "$USE_ROS" -eq 1 ]; then
    NO_ROS=0
fi

# Generate world if it doesn't exist or if requested
if [ "$REGENERATE" -eq 1 ] || [ ! -f "gazebo/worlds/generated_maze.world" ]; then
    echo "Generating maze world..."
    GEN_CMD="python3 gazebo_gen.py --rows $ROWS --cols $COLS --phase $PHASE"
    
    if [ -n "$OBJECTS_FILE" ]; then
        GEN_CMD="$GEN_CMD --objects $OBJECTS_FILE"
    fi
    
    if [ "$NO_ROS" -eq 1 ]; then
        echo "Using drone model without ROS plugins (avoids missing plugin errors)"
        $GEN_CMD --no-ros
    else
        echo "Using drone model with ROS plugins"
        $GEN_CMD
    fi
fi

echo ""
echo "Launching Gazebo..."
echo "Press Ctrl+C to stop"
echo ""

gz sim gazebo/worlds/generated_maze.world
