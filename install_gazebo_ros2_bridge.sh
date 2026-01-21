#!/bin/bash
echo "Installing Gazebo-ROS2 bridge for ROS 2 Rolling..."
echo ""

# Check ROS version
if [ -f /opt/ros/rolling/setup.bash ]; then
    ROS_VERSION="rolling"
elif [ -f /opt/ros/jazzy/setup.bash ]; then
    ROS_VERSION="jazzy"
elif [ -f /opt/ros/humble/setup.bash ]; then
    ROS_VERSION="humble"
else
    echo "Error: ROS 2 not found. Please install ROS 2 first."
    exit 1
fi

echo "Detected ROS 2 version: $ROS_VERSION"
echo ""
echo "Installing packages..."
sudo apt update
sudo apt install -y ros-${ROS_VERSION}-gz-ros2-interfaces

echo ""
echo "Installation complete!"
echo ""
echo "The plugins should now work. Try running the simulation again:"
echo "  ./run_sim.sh"
