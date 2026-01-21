#!/bin/bash
# Script to set up ROS 2 bridges for Gazebo simulation
# Run this after launching Gazebo to bridge topics between Gazebo and ROS 2

echo "=== Setting up ROS 2 bridges for Gazebo ==="
echo ""

# Check ROS 2
if [ -f /opt/ros/jazzy/setup.bash ]; then
    source /opt/ros/jazzy/setup.bash
    echo "✓ ROS 2 Jazzy loaded"
elif [ -f /opt/ros/rolling/setup.bash ]; then
    source /opt/ros/rolling/setup.bash
    echo "✓ ROS 2 Rolling loaded"
elif [ -f /opt/ros/humble/setup.bash ]; then
    source /opt/ros/humble/setup.bash
    echo "✓ ROS 2 Humble loaded"
else
    echo "✗ ROS 2 not found. Please install ROS 2 first."
    exit 1
fi

echo ""
echo "Available Gazebo topics (check with: gz topic -l):"
echo "  - Camera: /world/maze_world/model/quadrotor/link/base_link/sensor/camera/image"
echo "  - Odometry: /odom"
echo "  - Cmd Vel: /cmd_vel (publish from ROS 2)"
echo ""

echo "Setting up bridges..."
echo ""

# Bridge camera image
echo "1. Bridging camera image..."
ros2 run ros_gz_image image_bridge /world/maze_world/model/quadrotor/link/base_link/sensor/camera/image /camera/image_raw &
CAMERA_BRIDGE_PID=$!

# Bridge odometry
echo "2. Bridging odometry..."
ros2 run ros_gz_bridge parameter_bridge /odom@nav_msgs/msg/Odometry@gz.msgs.Odometry &
ODOM_BRIDGE_PID=$!

# Bridge cmd_vel (ROS 2 -> Gazebo)
echo "3. Bridging cmd_vel..."
ros2 run ros_gz_bridge parameter_bridge /cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist &
CMD_VEL_BRIDGE_PID=$!

echo ""
echo "✓ Bridges started (PIDs: $CAMERA_BRIDGE_PID, $ODOM_BRIDGE_PID, $CMD_VEL_BRIDGE_PID)"
echo ""
echo "To stop bridges, run:"
echo "  kill $CAMERA_BRIDGE_PID $ODOM_BRIDGE_PID $CMD_VEL_BRIDGE_PID"
echo ""
echo "Note: Topic names may vary. Check with:"
echo "  gz topic -l    (Gazebo topics)"
echo "  ros2 topic list (ROS 2 topics)"
echo ""

# Keep script running
wait
