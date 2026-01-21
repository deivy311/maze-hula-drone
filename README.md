# Drone Maze Navigation Simulation

A Python 3.10+ project that simulates a drone navigating a grid maze using a downward-facing camera that detects ArUco markers placed on the floor.

**Now supports Gazebo simulation!** See [GAZEBO_SETUP.md](GAZEBO_SETUP.md) for Gazebo setup instructions.

## Physical Environment

- **Maze Structure**: Grid of cells separated by thin metallic columns at grid corners
- **Walls**: White wall segments (rectangular barriers) between columns along cell boundaries
- **Markers**: 4 ArUco markers per cell arranged in a centered 2x2 pattern on the floor
- **Marker Layout**: Markers separated by black gaps; gap length = 0.5 * markerSize

## Features

- **Perfect Maze Generation**: Uses randomized DFS/backtracking to create a perfect maze
- **Geometric Collision Detection**: Continuous collision checking for walls (rectangles) and columns (circles)
- **Drone Kinematics**: Simulates drone motion with continuous position, yaw, and velocity control
- **ArUco Marker Detection**: Places ArUco markers on floor in 2x2 pattern and detects them via synthetic camera
- **Synthetic Camera**: Generates realistic camera frames with perspective projection
- **Real-time Visualization**: Shows top-down map view and camera feed

## Requirements

- Python 3.10+
- Ubuntu (or any Linux distribution with Python 3.10+)

## Installation

1. Create and activate virtual environment:
```bash
python3 -m venv venv
source venv/bin/activate
```

2. Install dependencies:
```bash
pip install -r requirements.txt
```

## Usage

### Basic Simulation

Run a default 5x5 maze simulation:
```bash
python main.py
```

### Custom Parameters

Run with custom parameters:
```bash
python main.py --rows 10 --cols 10 --cellSize 1.5 --markerSize 0.15 --z 1.5 --seconds 120 --seed 42
```

### Headless Mode

Run without visualization:
```bash
python main.py --no-show
```

### Command Line Options

- `--rows`: Number of maze rows (default: 5)
- `--cols`: Number of maze columns (default: 5)
- `--cellSize`: Size of each cell in meters (default: 1.0)
- `--markerSize`: Size of ArUco markers in meters (default: 0.1)
- `--colRadius`: Radius of columns in meters (default: 0.05)
- `--wallThickness`: Thickness of wall segments in meters (default: 0.02)
- `--droneRadius`: Drone footprint radius in meters (default: 0.1)
- `--z`: Drone altitude in meters (default: 1.0)
- `--seed`: Random seed for maze generation (optional)
- `--seconds`: Simulation duration in seconds (default: 60)
- `--show`: Show visualization (default: True)
- `--no-show`: Disable visualization

## Project Structure

- **`maze.py`**: Maze generation using DFS/backtracking
- **`geometry.py`**: Geometric representation of walls and columns with collision detection
- **`markers.py`**: ArUco marker placement in 2x2 pattern and ID encoding/decoding
- **`drone.py`**: Drone kinematic state and cell-to-cell controller
- **`camera_sim.py`**: Synthetic camera with perspective projection
- **`detect.py`**: ArUco marker detection wrapper
- **`main.py`**: Simulation loop with visualization

## ArUco Marker ID Encoding

Each cell has 4 markers arranged in a 2x2 pattern:
- **Quadrant 0**: Top-left
- **Quadrant 1**: Top-right
- **Quadrant 2**: Bottom-left
- **Quadrant 3**: Bottom-right

ID encoding: `base * 10 + quadrant`
- `base = r * cols + c`
- `quadrant = 0, 1, 2, or 3`

## Marker Placement

Markers are placed on the floor (z=0) in each cell:
- Total footprint: `2 * markerSize + gap` (where `gap = 0.5 * markerSize`)
- Marker centers at offsets: `±(markerSize/2 + gap/2)` from cell center

## Example

```bash
# Run a 5x5 maze for 60 seconds
python main.py --rows 5 --cols 5

# Run a larger maze with custom parameters
python main.py --rows 8 --cols 8 --cellSize 1.2 --markerSize 0.12 --z 1.5 --seed 123 --seconds 120
```

The simulation will:
1. Generate a perfect maze with walls and columns
2. Place ArUco markers on the floor in a 2x2 pattern per cell
3. Start the drone at (0,0)
4. Run the simulation loop at 30 Hz:
   - Update drone kinematics with collision checking
   - Generate camera frame
   - Detect ArUco markers
   - Update controller
   - Visualize (if enabled)

## Visualization

When visualization is enabled, you'll see:
- **Left panel**: Top-down map view showing:
  - Maze walls (black lines)
  - Columns (gray circles)
  - Drone position (red dot)
  - Drone heading (red arrow)
- **Right panel**: Camera view showing:
  - Synthetic camera feed
  - Detected markers (highlighted)

## Notes

- The drone starts at cell (0,0) and attempts to navigate to the bottom-right corner
- Collision detection prevents the drone from crossing walls or hitting columns
- Camera simulation includes perspective projection and marker warping
- The controller uses detected markers to infer current cell position
- Detection rate is logged and displayed
