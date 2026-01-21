# ROS 2 Jazzy Setup for Gazebo Jazzy

## You're using ROS 2 Jazzy - Perfect!

ROS 2 Jazzy is the recommended version for Gazebo Jazzy (gz-jetty). Here's how to complete the setup:

## Current Status

✅ ROS 2 Jazzy installed  
✅ Gazebo Jazzy (gz-jetty) installed  
⚠️ Gazebo-ROS2 bridge needs to be installed

## Install Gazebo-ROS2 Bridge

### Option 1: Use the install script
```bash
./install_jazzy_bridge.sh
```

### Option 2: Manual installation
```bash
sudo apt update
sudo apt install ros-jazzy-ros-gz-bridge ros-jazzy-ros-gz-interfaces ros-jazzy-ros-gz-sim
```

These are the correct packages for ROS 2 Jazzy:
- `ros-jazzy-ros-gz-bridge` - Bridge between ROS 2 and Gazebo
- `ros-jazzy-ros-gz-interfaces` - Message interfaces
- `ros-jazzy-ros-gz-sim` - Simulation tools

## Running the Simulation

### Step 1: Setup and Launch Gazebo
```bash
source /opt/ros/jazzy/setup.bash
./run_sim.sh
```

The script will automatically:
- Source ROS 2 Jazzy
- Set Gazebo model paths
- Generate the maze world (if needed)
- Launch Gazebo

### Step 2: Run Controller (Optional, in another terminal)
```bash
cd ~/Repos/maze-hula-drone
source /opt/ros/jazzy/setup.bash
python3 gazebo_interface_ros2.py
```

Or use the main simulation script:
```bash
python3 main_gazebo.py --rows 5 --cols 5 --seconds 60
```

## Verify ROS 2 Topics

After launching Gazebo, check available topics:
```bash
source /opt/ros/jazzy/setup.bash
ros2 topic list
```

You should see:
- `/camera/image_raw` - Camera feed
- `/odom` - Odometry
- `/cmd_vel` - Velocity commands

## Troubleshooting

### Plugins still not loading
The plugin names in the drone model might need adjustment. Check:
- `gz-ros2-camera`
- `gz-ros2-odometry-publisher`  
- `gz-ros2-cmd-vel`

These might be named differently in Jazzy. Check available plugins:
```bash
gz plugin --list
```

### No topics visible
1. Make sure Gazebo is running
2. Source ROS 2: `source /opt/ros/jazzy/setup.bash`
3. Check if bridge is installed: `dpkg -l | grep gz-ros2`

## Next Steps

Once the bridge is installed:
1. Regenerate world with ArUco textures: `python3 gazebo_gen.py --rows 5 --cols 5`
2. Launch: `./run_sim.sh`
3. You should see ArUco patterns on markers (not just black boxes)
4. ROS 2 topics should be available for control
