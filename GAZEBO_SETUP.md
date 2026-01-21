# Gazebo Setup Instructions

## Prerequisites

1. **Install Gazebo Jazzy (gz-jetty)** (if not already installed):
```bash
sudo apt-get install gz-jetty
```

2. **Install ROS 2** (for Python interface):
```bash
# For Ubuntu 22.04 (ROS 2 Humble) or Ubuntu 24.04 (ROS 2 Jazzy)
# See: https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debians.html
```

3. **Install ROS 2 packages**:
```bash
sudo apt install ros-humble-ros-base ros-humble-cv-bridge ros-humble-image-transport
# Or for ROS 2 Jazzy:
sudo apt install ros-jazzy-ros-base ros-jazzy-cv-bridge ros-jazzy-image-transport
```

4. **Install Gazebo ROS 2 integration**:
```bash
sudo apt install ros-humble-gz-ros2-interfaces
# Or for Jazzy:
sudo apt install ros-jazzy-gz-ros2-interfaces
```

## Setup

1. **Set Gazebo model path**:
```bash
export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:$(pwd)/gazebo/models
```

2. **Generate the maze world**:
```bash
python gazebo_gen.py --rows 5 --cols 5 --out gazebo/worlds/maze.world
```

3. **Launch Gazebo Jazzy**:
```bash
gz sim gazebo/worlds/maze.world
```

Or with ROS 2:
```bash
ros2 run gz_ros2_interface create gz sim gazebo/worlds/maze.world
```

## Running the Simulation

1. **Source ROS 2** (if using ROS 2 interface):
```bash
source /opt/ros/humble/setup.bash  # Or ros-jazzy for Jazzy
```

2. **Launch Gazebo Jazzy**:
```bash
gz sim gazebo/worlds/maze.world
```

3. **In another terminal, run the Python controller**:
```bash
source /opt/ros/humble/setup.bash  # Adjust for your ROS 2 version
python gazebo_interface.py
```

## Alternative: Direct Gazebo API (without ROS)

If you prefer not to use ROS, you can use Gazebo's Python API directly:

```python
from gazebo_msgs.srv import GetModelState, SetModelState
import rospy

rospy.init_node('gazebo_interface')
get_state = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)
```

## Notes

- The ArUco markers are currently represented as simple black boxes. For actual ArUco detection, you may want to:
  - Use a Gazebo plugin that renders ArUco markers
  - Or use image textures with ArUco patterns
  - Or use the `gazebo_aruco_plugin` if available

- The drone model is simplified. For more realistic physics, consider using:
  - `hector_quadrotor` package
  - `rotors_simulator` package
  - Or other quadrotor models
