# Quick Start Guide for Gazebo Jazzy (gz-jetty)

## You've installed `gz-jetty` - Great!

Gazebo Jazzy uses a different command structure than classic Gazebo. Here's how to use it:

## Setup

1. **Set model path** (run this first!):
```bash
source setup_gazebo_path.sh
```

Or manually:
```bash
export GZ_SIM_RESOURCE_PATH=$GZ_SIM_RESOURCE_PATH:$(pwd)/gazebo/models
```

**Important**: You must set this path before launching Gazebo, otherwise models won't be found!

2. **Generate the maze world**:
```bash
python gazebo_gen.py --rows 5 --cols 5
```

3. **Launch Gazebo Jazzy** (in the same terminal where you set the path):
```bash
gz sim gazebo/worlds/generated_maze.world
```

## Using ROS 2 (Optional)

If you want to use ROS 2 for control:

1. **Install ROS 2** (if not already installed):
```bash
# For Ubuntu 22.04
sudo apt install ros-humble-desktop
# Or for Ubuntu 24.04
sudo apt install ros-jazzy-desktop
# Or for Rolling (latest)
sudo apt install ros-rolling-desktop
```

2. **Install Gazebo-ROS2 bridge**:
```bash
sudo apt install ros-humble-gz-ros2-interfaces
# Or for Jazzy:
sudo apt install ros-jazzy-gz-ros2-interfaces
# Or for Rolling:
sudo apt install ros-rolling-gz-ros2-interfaces
```

3. **Source ROS 2**:
```bash
source /opt/ros/humble/setup.bash
# Or for Jazzy:
source /opt/ros/jazzy/setup.bash
# Or for Rolling:
source /opt/ros/rolling/setup.bash
```

4. **Launch Gazebo with ROS 2 bridge**:
```bash
gz sim gazebo/worlds/generated_maze.world
```

5. **In another terminal, run the controller**:
```bash
source /opt/ros/humble/setup.bash  # Adjust for your version
python gazebo_interface_ros2.py
```

## Direct Gazebo Jazzy API (Without ROS)

You can also control Gazebo Jazzy directly using its Python API:

```python
from gz.msgs10.double_pb2 import Double
from gz.transport13 import Node as TransportNode

# Create transport node
node = TransportNode()
pub = node.advertise("/model/quadrotor/joint/rotor_0/cmd_vel", Double)
```

## Key Differences from Classic Gazebo

- Use `gz sim` instead of `gazebo`
- Use `GZ_SIM_RESOURCE_PATH` instead of `GAZEBO_MODEL_PATH`
- Plugin names are different (e.g., `gz-ros2-camera` instead of `libgazebo_ros_camera.so`)
- ROS 2 topics may have different names

## Troubleshooting

1. **Models not found**: Make sure `GZ_SIM_RESOURCE_PATH` is set correctly
2. **Plugins not working**: Check that you have the right Gazebo Jazzy plugins installed
3. **ROS 2 topics not visible**: Make sure the ROS 2 bridge is running

## Next Steps

- Check the generated world file: `gazebo/worlds/generated_maze.world`
- Customize the drone model in: `gazebo/models/drone/`
- Adjust maze parameters in: `gazebo_gen.py`
