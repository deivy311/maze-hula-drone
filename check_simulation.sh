#!/bin/bash
# Script to check if simulation is ready

echo "=== Checking Simulation Status ==="
echo ""

# Check Gazebo
echo "1. Checking Gazebo..."
if pgrep -f "gz sim" > /dev/null; then
    echo "   ✓ Gazebo is running"
else
    echo "   ✗ Gazebo is NOT running"
    echo "   → Start with: gz sim gazebo/worlds/generated_maze.world"
fi

# Check ROS 2
echo ""
echo "2. Checking ROS 2..."
if [ -f /opt/ros/jazzy/setup.bash ]; then
    source /opt/ros/jazzy/setup.bash 2>/dev/null
    echo "   ✓ ROS 2 Jazzy found"
elif [ -f /opt/ros/rolling/setup.bash ]; then
    source /opt/ros/rolling/setup.bash 2>/dev/null
    echo "   ✓ ROS 2 Rolling found"
elif [ -f /opt/ros/humble/setup.bash ]; then
    source /opt/ros/humble/setup.bash 2>/dev/null
    echo "   ✓ ROS 2 Humble found"
else
    echo "   ✗ ROS 2 not found"
    exit 1
fi

# Check ROS topics
echo ""
echo "3. Checking ROS 2 topics..."
if command -v ros2 > /dev/null; then
    TOPICS=$(ros2 topic list 2>/dev/null)
    if echo "$TOPICS" | grep -q "/cmd_vel"; then
        echo "   ✓ /cmd_vel topic exists"
    else
        echo "   ✗ /cmd_vel topic NOT found"
        echo "   → Run: source setup_ros2_bridge.sh"
    fi
    
    if echo "$TOPICS" | grep -q "/odom"; then
        echo "   ✓ /odom topic exists"
    else
        echo "   ✗ /odom topic NOT found"
        echo "   → Run: source setup_ros2_bridge.sh"
    fi
    
    if echo "$TOPICS" | grep -q "/camera"; then
        echo "   ✓ Camera topic exists"
    else
        echo "   ⚠ Camera topic not found (optional)"
    fi
else
    echo "   ✗ ros2 command not available"
fi

# Check bridge processes
echo ""
echo "4. Checking bridge processes..."
if pgrep -f "ros_gz_bridge" > /dev/null || pgrep -f "ros_gz_image" > /dev/null; then
    echo "   ✓ Bridge processes are running"
else
    echo "   ✗ Bridge processes NOT running"
    echo "   → Run: source setup_ros2_bridge.sh"
fi

echo ""
echo "=== Summary ==="
if pgrep -f "gz sim" > /dev/null && pgrep -f "ros_gz" > /dev/null; then
    echo "✓ Simulation appears to be ready!"
    echo ""
    echo "To run your code:"
    echo "  cd code/XDrone"
    echo "  USE_SIMULATION=true python3 main.py"
else
    echo "✗ Simulation is NOT ready"
    echo ""
    echo "Follow these steps:"
    echo "  1. Terminal 1: gz sim gazebo/worlds/generated_maze.world"
    echo "  2. Terminal 2: source setup_ros2_bridge.sh"
    echo "  3. Terminal 3: cd code/XDrone && USE_SIMULATION=true python3 main.py"
fi
