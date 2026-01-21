#!/bin/bash
# Test ROS 2 setup for Gazebo

echo "Testing ROS 2 setup..."

# Source ROS 2
if [ -f /opt/ros/rolling/setup.bash ]; then
    source /opt/ros/rolling/setup.bash
    echo "✓ ROS 2 Rolling sourced"
elif [ -f /opt/ros/jazzy/setup.bash ]; then
    source /opt/ros/jazzy/setup.bash
    echo "✓ ROS 2 Jazzy sourced"
elif [ -f /opt/ros/humble/setup.bash ]; then
    source /opt/ros/humble/setup.bash
    echo "✓ ROS 2 Humble sourced"
else
    echo "✗ ROS 2 not found. Please install ROS 2 first."
    exit 1
fi

# Check for Gazebo-ROS2 packages
echo "Checking for Gazebo-ROS2 packages..."
if dpkg -l | grep -q "ros-.*-gz-ros2"; then
    echo "✓ Gazebo-ROS2 packages found"
else
    echo "⚠ Gazebo-ROS2 packages not found. Install with:"
    if [ -f /opt/ros/jazzy/setup.bash ]; then
        echo "  sudo apt install ros-jazzy-gz-ros2-control"
        echo "  ./install_jazzy_bridge.sh"
    elif [ -f /opt/ros/rolling/setup.bash ]; then
        echo "  sudo apt install ros-rolling-gz-ros2-interfaces"
    else
        echo "  sudo apt install ros-humble-gz-ros2-interfaces"
    fi
fi

# Check Gazebo
if command -v gz &> /dev/null; then
    echo "✓ Gazebo found: $(gz --version 2>&1 | head -1)"
else
    echo "✗ Gazebo not found"
fi

echo ""
echo "Setup complete! You can now:"
echo "  1. Generate world: python gazebo_gen.py --rows 5 --cols 5"
echo "  2. Launch Gazebo: gz sim gazebo/worlds/generated_maze.world"
echo "  3. Run controller: python gazebo_interface_ros2.py"
