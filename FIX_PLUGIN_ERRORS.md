# Fixing ROS 2 Plugin Errors

## The Errors You're Seeing

```
Failed to load system plugin [gz-ros2-odometry-publisher] : Could not find shared library.
Failed to load system plugin [gz-ros2-cmd-vel] : Could not find shared library.
Failed to load system plugin [gz-ros2-camera] : Could not find shared library.
```

## What This Means

These errors occur because the Gazebo-ROS2 bridge packages aren't installed. **The simulation will still work** - you just won't have ROS 2 topics for camera, odometry, and control.

## Solution: Install the Bridge

### Option 1: Use the Install Script
```bash
./install_gazebo_ros2_bridge.sh
```

### Option 2: Manual Installation

For ROS 2 Rolling:
```bash
sudo apt update
sudo apt install ros-rolling-gz-ros2-interfaces
```

For ROS 2 Jazzy:
```bash
sudo apt install ros-jazzy-gz-ros2-interfaces
```

For ROS 2 Humble:
```bash
sudo apt install ros-humble-gz-ros2-interfaces
```

## After Installation

1. **Restart Gazebo** - The plugins should now load without errors
2. **Check ROS 2 topics** (in another terminal):
   ```bash
   source /opt/ros/rolling/setup.bash
   ros2 topic list
   ```
   
   You should see:
   - `/camera/image_raw`
   - `/odom`
   - `/cmd_vel`

## If You Don't Need ROS 2

If you just want to visualize the simulation without ROS 2, you can:

1. Use the world file as-is (errors are harmless)
2. Or modify the drone model to remove ROS 2 plugins (see `gazebo/models/drone/model_no_ros.sdf`)

## Verify Installation

After installing, run:
```bash
./test_ros2_setup.sh
```

This will check if everything is set up correctly.
