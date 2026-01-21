# How to Run the Simulation

## Quick Start (3 Steps)

### Step 1: Setup Environment
Open a terminal and run:
```bash
cd ~/Repos/maze-hula-drone
source /opt/ros/rolling/setup.bash
source setup_gazebo_path.sh
```

### Step 2: Generate the Maze World
```bash
python gazebo_gen.py --rows 5 --cols 5
```

This creates `gazebo/worlds/generated_maze.world` with:
- A 5x5 perfect maze
- Columns at grid vertices
- Wall segments
- 100 ArUco markers (4 per cell in 2x2 pattern)
- A drone at the starting position

### Step 3: Launch Gazebo
```bash
gz sim gazebo/worlds/generated_maze.world
```

You should see the Gazebo GUI with the maze, columns, walls, markers, and drone.

## Running with ROS 2 Control (Optional)

If you want to control the drone via ROS 2:

### Terminal 1: Launch Gazebo
```bash
cd ~/Repos/maze-hula-drone
source /opt/ros/rolling/setup.bash
source setup_gazebo_path.sh
gz sim gazebo/worlds/generated_maze.world
```

### Terminal 2: Run the Controller
```bash
cd ~/Repos/maze-hula-drone
source /opt/ros/rolling/setup.bash
python gazebo_interface_ros2.py
```

Or use the main simulation script:
```bash
python main_gazebo.py --rows 5 --cols 5 --seconds 60
```

## Using run_sim.sh (Recommended)

By default, `run_sim.sh` runs **without ROS plugins** to avoid missing plugin errors:
```bash
./run_sim.sh --regenerate
```

If you want ROS plugins (camera/odom/cmd_vel), use:
```bash
./run_sim.sh --regenerate --ros
```

## Customizing the Maze

You can customize the maze when generating:

```bash
python gazebo_gen.py \
  --rows 10 \
  --cols 10 \
  --cellSize 1.5 \
  --markerSize 0.15 \
  --colRadius 0.08 \
  --wallThickness 0.03 \
  --seed 42
```

Parameters:
- `--rows`: Number of rows (default: 5)
- `--cols`: Number of columns (default: 5)
- `--cellSize`: Size of each cell in meters (default: 1.0)
- `--markerSize`: ArUco marker size in meters (default: 0.1)
- `--colRadius`: Column radius in meters (default: 0.05)
- `--wallThickness`: Wall thickness in meters (default: 0.02)
- `--seed`: Random seed for reproducibility (optional)

## Troubleshooting

### Models not found
Make sure you ran `source setup_gazebo_path.sh` before launching Gazebo.

### ROS 2 plugins not loading
Install the bridge:
```bash
sudo apt install ros-rolling-gz-ros2-interfaces
```

### Gazebo doesn't start
Check if Gazebo is installed:
```bash
gz --version
```

### No visualization
Make sure you have a display/X11 forwarding if using SSH.

## What You'll See

- **Maze**: Grid of cells with walls
- **Columns**: Gray cylinders at grid vertices
- **Walls**: White rectangular barriers
- **Markers**: Black squares on the floor (ArUco markers in 2x2 pattern per cell)
- **Drone**: Red cylinder at starting position (0,0)

## Next Steps

- View camera feed: Check ROS 2 topics with `ros2 topic list`
- Control drone: Publish to `/cmd_vel` topic
- Detect markers: Use the Python interface to detect ArUco markers from camera
