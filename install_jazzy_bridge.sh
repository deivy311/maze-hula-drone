#!/bin/bash
# Install Gazebo-ROS2 bridge for ROS 2 Jazzy

echo "Installing Gazebo-ROS2 bridge for ROS 2 Jazzy..."
echo ""

# Check if Jazzy is installed
if [ ! -f /opt/ros/jazzy/setup.bash ]; then
    echo "Error: ROS 2 Jazzy not found. Please install it first."
    exit 1
fi

echo "ROS 2 Jazzy detected. Installing bridge packages..."
echo ""

sudo apt update
sudo apt install -y ros-jazzy-ros-gz-bridge ros-jazzy-ros-gz-interfaces ros-jazzy-ros-gz-sim

echo ""
echo "Installation complete!"
echo ""
echo "The Gazebo-ROS2 plugins should now work. Try running the simulation:"
echo "  ./run_sim.sh"
echo ""
echo "To verify, check ROS 2 topics after launching Gazebo:"
echo "  source /opt/ros/jazzy/setup.bash"
echo "  ros2 topic list"
