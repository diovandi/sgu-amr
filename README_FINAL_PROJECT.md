# Final Project: Domestic Mobile Robot with Nav2 & SLAM

**Course:** Autonomous Mobile Robot (2025-2026/2025-1-2943)  
**Instructor:** Dr. Rusman Rusyadi  
**Institution:** Swiss German University  
**Author:** Diovandi Basheera Putra  
**Branch:** `final-project`

---

## Project Overview

This project implements a complete autonomous navigation system for a domestic mobile robot operating in indoor environments. The system progresses through six phases from basic simulation to autonomous patrol with comprehensive performance analysis.

**Key Features:**
- Custom differential drive robot design (`rudimentary_bot`)
- SLAM-based mapping using SLAM Toolbox
- AMCL particle filter localization
- Nav2 path planning and control
- Autonomous waypoint-based patrol
- Offline trajectory analysis and visualization

---

## Table of Contents

- [System Requirements](#system-requirements)
- [Quick Start](#quick-start)
- [Phase-by-Phase Guide](#phase-by-phase-guide)
- [Directory Structure](#directory-structure)
- [Key Files](#key-files)
- [LaTeX Report Compilation](#latex-report-compilation)
- [Troubleshooting](#troubleshooting)
- [Technical Details](#technical-details)
- [Known Limitations](#known-limitations)

---

## System Requirements

### Hardware
- Modern x86-64 CPU (multi-core recommended)
- 8+ GB RAM
- NVIDIA GPU (optional, for Gazebo performance)
- ~50 GB disk space (for ROS 2, Gazebo, and workspace)

### Software
- **OS:** Ubuntu 24.04 LTS or Pop!\_OS 24.04 (tested)
- **ROS 2:** Jazzy Jalisco
- **Gazebo:** Harmonic
- **Python:** 3.12+

### Required ROS 2 Packages
```bash
sudo apt-get install \
  ros-jazzy-desktop \
  ros-jazzy-gazebo-ros-pkgs \
  ros-jazzy-ros-gz \
  ros-jazzy-navigation2 \
  ros-jazzy-nav2-bringup \
  ros-jazzy-slam-toolbox \
  ros-jazzy-robot-localization \
  ros-jazzy-teleop-twist-keyboard \
  ros-jazzy-plotjuggler-ros \
  ros-jazzy-tf2-tools
```

### For LaTeX Report Compilation
```bash
sudo apt-get install texlive-full  # Or minimal: texlive-latex-extra texlive-science
```

---

## Quick Start

### 1. Clone and Build

```bash
# Clone repository (if not already cloned)
cd ~/ros2_ws/src
# [Clone command depends on your git setup]

# Build workspace
cd ~/ros2_ws
source /opt/ros/jazzy/setup.bash
colcon build --packages-select exercise_launch

# Source workspace
source install/setup.bash
```

### 2. Run Navigation Demo

```bash
# Launch complete navigation system
./src/exercise_launch/scripts/run_navigation.sh

# In RViz:
# 1. Wait for map to load (~10 seconds)
# 2. Use "2D Goal Pose" tool to click destination
# 3. Robot will plan and execute autonomous navigation
```

### 3. View LaTeX Report

```bash
cd docs
./compile_report.sh
# PDF output: docs/compiled/final_project_report.pdf
```

---

## Phase-by-Phase Guide

### Phase 1: Simulation and Robot Design

**Objective:** Spawn custom robot in Gazebo with proper sensor configuration

```bash
./src/exercise_launch/scripts/run_simulation.sh
```

**What it does:**
- Launches Gazebo Harmonic with `home.sdf` world
- Spawns `rudimentary_bot` robot
- Bridges topics: /cmd\_vel, /odom, /scan, /imu
- Starts robot\_state\_publisher for TF tree

**Key files:**
- `src/exercise_launch/urdf/rudimentary_bot.urdf.xacro` - Robot URDF
- `src/exercise_launch/launch/simulation.launch.py` - Base simulation launch
- `src/exercise_launch/worlds/home.sdf` - House environment

---

### Phase 2: LIDAR Processing

**Objective:** Read and visualize LIDAR data with proper QoS

```bash
./src/exercise_launch/scripts/run_phase2.sh
```

**What it does:**
- Includes Phase 1 simulation
- Launches LIDAR reader node with Best Effort QoS
- Opens RViz with robot model and scan visualization

**Key accomplishments:**
- QoS profile matching (Best Effort for sensors)
- False detection mitigation via mechanical design
- Statistical data validation

---

### Phase 3: SLAM-Based Mapping

**Objective:** Generate occupancy grid map of house environment

```bash
# Launch SLAM system
./src/exercise_launch/scripts/run_slam.sh

# In separate terminal, drive robot manually
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -p stamped:=true

# After mapping, save the map
./src/exercise_launch/scripts/save_map.sh
```

**What it does:**
- Includes Phase 1 simulation
- Runs EKF for sensor fusion (odom + IMU)
- Runs SLAM Toolbox (async\_online\_mapper)
- Opens RViz with map visualization

**Outputs:**
- `maps/house_map.pgm` - Occupancy grid image
- `maps/house_map.yaml` - Map metadata

**Key accomplishments:**
- Complete house map generation
- Loop closure detection
- Lifecycle node management
- Front caster wheel addition (prevents false walls)

---

### Phase 4: Autonomous Navigation

**Objective:** Navigate autonomously to operator-specified goals

```bash
./src/exercise_launch/scripts/run_navigation.sh
```

**What it does:**
- Includes Phase 1 simulation
- Runs EKF for sensor fusion
- Runs Nav2 full stack:
  - Map server (loads house\_map)
  - AMCL (particle filter localization)
  - Planner server (global path planning)
  - Controller server (Regulated Pure Pursuit)
  - Recovery behaviors
- Opens RViz with navigation tools
- Goal bridge for click-to-navigate

**How to use:**
1. Wait ~15 seconds for all nodes to activate
2. In RViz, use "2D Goal Pose" tool
3. Click desired destination on map
4. Robot plans and executes path autonomously

**Key accomplishments:**
- AMCL tuned for stable localization (1000-3000 particles)
- Costmaps configured (robot\_radius=0.25m, inflation=0.45m)
- Regulated Pure Pursuit controller (specification requirement)
- Multi-goal navigation capability

---

### Phase 5: Autonomous Patrol Recording

**Objective:** Execute waypoint-based patrol and record data

**Step 1: Record waypoints**
```bash
./src/exercise_launch/scripts/run_patrol_waypoint_recorder.sh
# In RViz, use "Publish Point" tool to click 3 waypoints:
# 1st click = Kitchen
# 2nd click = Bedroom
# 3rd click = Home (starting position)
```

**Step 2: Execute and record patrol**
```bash
./src/exercise_launch/scripts/run_record_patrol.sh
```

**What it does:**
- Runs navigation system headless (no RViz for performance)
- Executes autonomous patrol through 3 waypoints
- Records bag with: /odom, /odometry/filtered, /amcl\_pose, /scan, /plan, /cmd\_vel, /tf

**Outputs:**
- `exercise4_recording/` directory with .mcap bag file

---

### Phase 6: Performance Analysis

**Objective:** Analyze navigation performance through trajectory comparison

```bash
./src/exercise_launch/scripts/run_analyze_patrol.sh

# In separate terminal, launch PlotJuggler
ros2 run plotjuggler plotjuggler
# In PlotJuggler: Streaming → ROS 2 Topic Subscriber
# Select /planned_xy and /actual_xy for comparison
```

**What it does:**
- Replays recorded bag with simulated time
- Publishes static map for reference
- Runs analysis node to transform odometry to map frame
- Visualizes actual vs planned paths in RViz
- Streams XY data to PlotJuggler

**Key accomplishments:**
- Offline trajectory reconstruction in map frame
- Path tracking quantification (0.15m mean deviation)
- Localization drift characterization
- Fixed-joint URDF for clean playback visualization

---

## Directory Structure

```
ros2_ws/
├── src/
│   └── exercise_launch/              # Main package
│       ├── launch/                   # Launch files for each phase
│       │   ├── simulation.launch.py
│       │   ├── phase2.launch.py
│       │   ├── slam.launch.py
│       │   ├── navigation.launch.py
│       │   ├── record_patrol.launch.py
│       │   └── analyze_patrol.launch.py
│       ├── config/                   # YAML parameter files
│       │   ├── gz_bridge.yaml
│       │   ├── ekf_slam_params.yaml
│       │   ├── slam_params.yaml
│       │   └── nav2_params.yaml
│       ├── urdf/                     # Robot description files
│       │   ├── rudimentary_bot.urdf.xacro
│       │   └── rudimentary_bot_analysis.urdf.xacro
│       ├── rviz/                     # RViz configurations
│       ├── worlds/                   # Gazebo world files
│       │   └── home.sdf
│       ├── scripts/                  # Wrapper scripts
│       │   ├── run_simulation.sh
│       │   ├── run_phase2.sh
│       │   ├── run_slam.sh
│       │   ├── run_navigation.sh
│       │   ├── run_record_patrol.sh
│       │   └── run_analyze_patrol.sh
│       └── exercise_launch/          # Python nodes
│           ├── lidar_reader.py
│           ├── nav2_goal_bridge.py
│           ├── patrol_waypoint_recorder.py
│           ├── house_patrol.py
│           └── patrol_analysis.py
├── config/
│   └── patrol_waypoints.yaml        # Saved waypoints
├── maps/
│   ├── house_map.pgm                # Generated map
│   └── house_map.yaml               # Map metadata
├── docs/
│   ├── final_project_report.tex     # Comprehensive LaTeX report
│   ├── references.bib               # Bibliography
│   ├── preamble.tex                 # LaTeX preamble
│   ├── commands.tex                 # Custom LaTeX commands
│   ├── Makefile                     # Build system
│   ├── compile_report.sh            # Compilation script
│   ├── README_REPORT.md             # Report compilation guide
│   ├── screenshots/                 # Images (local only)
│   └── compiled/                    # Output directory
│       └── final_project_report.pdf
└── README_FINAL_PROJECT.md          # This file
```

---

## Key Files

### Configuration Files

| File | Purpose |
|------|---------|
| `config/ekf_slam_params.yaml` | EKF sensor fusion parameters |
| `config/slam_params.yaml` | SLAM Toolbox configuration |
| `config/nav2_params.yaml` | Complete Nav2 stack parameters |
| `config/gz_bridge.yaml` | Gazebo-ROS topic mappings |
| `config/patrol_waypoints.yaml` | Saved patrol waypoints |

### Launch Files

| File | Purpose |
|------|---------|
| `simulation.launch.py` | Base simulation (Gazebo + robot + bridge) |
| `slam.launch.py` | Simulation + EKF + SLAM Toolbox |
| `navigation.launch.py` | Simulation + EKF + Nav2 + AMCL |
| `record_patrol.launch.py` | Headless navigation + patrol + recording |
| `analyze_patrol.launch.py` | Bag playback + analysis + visualization |

### Robot Description

| File | Purpose |
|------|---------|
| `rudimentary_bot.urdf.xacro` | Main robot URDF (continuous wheel joints) |
| `rudimentary_bot_analysis.urdf.xacro` | Analysis URDF (fixed wheel joints for playback) |

---

## LaTeX Report Compilation

The comprehensive project report is written in LaTeX for professional presentation.

### Quick Compilation

```bash
cd docs
./compile_report.sh
```

Output: `docs/compiled/final_project_report.pdf`

### Using Make

```bash
cd docs
make          # Full compilation
make quick    # Fast recompile
make clean    # Clean auxiliary files
make view     # Open PDF
```

### Manual Compilation

```bash
cd docs
pdflatex final_project_report.tex
bibtex final_project_report
pdflatex final_project_report.tex
pdflatex final_project_report.tex
```

See `docs/README_REPORT.md` for detailed compilation instructions and troubleshooting.

---

## Troubleshooting

### GUI Crashes (Pop!\_OS / NVIDIA)

**Symptom:** Gazebo or RViz crashes immediately on launch

**Solution:** Use the provided wrapper scripts which configure X11/NVIDIA environment:
```bash
./src/exercise_launch/scripts/run_[phase].sh
```

These scripts set:
- `QT_QPA_PLATFORM=xcb` (force X11)
- NVIDIA-specific OpenGL settings
- `xhost +local:` (allow X11 connections)

### No LIDAR Data in Subscriber

**Symptom:** `/scan` topic exists but subscriber receives no data

**Solution:** Configure subscriber with **Best Effort** QoS to match Gazebo:
```python
from rclpy.qos import QoSProfile, ReliabilityPolicy

sensor_qos = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT, depth=10)
self.create_subscription(LaserScan, '/scan', callback, sensor_qos)
```

### RViz "No map received"

**Symptom:** Map server publishing but RViz shows no map

**Solution:** Configure RViz Map display with **Transient Local** durability:
- Map → Topic → Durability Policy → Transient Local
- Map Updates → Topic → Durability Policy → Transient Local

### SLAM Toolbox Not Publishing Map

**Symptom:** `/map` topic exists but no messages published

**Solution:** SLAM Toolbox is a lifecycle node requiring activation. Ensure lifecycle manager is running:
```yaml
slam_lifecycle_manager:
  parameters:
    autostart: true
    node_names: ['slam_toolbox']
```

### Robot Not Moving During Navigation

**Symptom:** Nav2 creates path but robot doesn't move

**Common causes:**
1. **Frame ID mismatch:** Ensure `base_frame_id: base_link` in all Nav2 params
2. **TF conflict:** Only one node should publish `odom → base_link` (EKF, not diff\_drive)
3. **Costmap transform issues:** Check `/tf` tree has complete `map → odom → base_link` chain

**Debug commands:**
```bash
ros2 run tf2_tools view_frames          # Visualize TF tree
ros2 topic echo /cmd_vel                # Check if commands published
ros2 node info /controller_server       # Check node status
```

### False Walls in SLAM

**Symptom:** Perpendicular walls appear in front of robot when braking/turning

**Solution:** This was solved by adding a **front caster wheel** to prevent forward pitch. Ensure URDF includes front caster at `(0.25, 0, 0.05)`.

### EKF Not Meeting Update Rate

**Symptom:** EKF logs "Failed to meet update rate"

**Solution:** Reduce frequency or queue sizes:
```yaml
frequency: 5.0           # Lower than 30Hz
sensor_timeout: 0.2      # Match frequency
odom0_queue_size: 5      # Smaller queue
```

---

## Technical Details

### Robot Specifications

| Parameter | Value |
|-----------|-------|
| Chassis | 0.5m × 0.3m × 0.15m box |
| Wheel radius | 0.1 m |
| Wheelbase | 0.35 m |
| Max linear velocity | 0.5 m/s |
| Max angular velocity | 1.0 rad/s |
| LIDAR range | 0.4 - 10.0 m |
| LIDAR FOV | 360° (2D scan) |
| LIDAR update rate | 10 Hz |
| IMU update rate | 100 Hz |

### Transform Frame Hierarchy

```
map
 └─ odom (published by AMCL)
     └─ base_link (published by EKF)
         ├─ laser_link (static, from URDF)
         ├─ imu_link (static, from URDF)
         ├─ left_wheel (from joint_state_publisher)
         ├─ right_wheel (from joint_state_publisher)
         ├─ caster_wheel (static, from URDF)
         └─ front_caster_wheel (static, from URDF)
```

### Critical Design Decisions

1. **Diff drive plugin:** `publish_odom_tf: false` (EKF handles TF)
2. **Front caster wheel:** Prevents pitch, eliminates false walls
3. **LIDAR position:** Centered (0,0,0.25) for symmetric coverage
4. **LIDAR min range:** 0.4m to prevent self-detection
5. **EKF frequency:** 5Hz (tested 30Hz, less stable)
6. **AMCL particles:** 1000-3000 (heavy setting for robustness)
7. **Controller:** Regulated Pure Pursuit (project requirement)

### Nav2 Specification Requirements

The following parameters were explicitly required:
- ✅ `robot_radius: 0.25` m
- ✅ `inflation_radius: 0.45` m (door clearance)
- ✅ `cost_scaling_factor: 5.0`
- ✅ Controller: Regulated Pure Pursuit
- ✅ `use_sim_time: true` throughout

---

## Known Limitations

### Localization Drift

The `map → odom` transform exhibits jitter, especially after wheel slip. This is a known characteristic of particle filter localization and was noted as requiring additional tuning.

**Current performance:**
- Mean path deviation: ~0.15 m
- Max deviation: ~0.35 m (long corridors)
- Recovery time after collision: 2-3 seconds

**Future improvements:**
- Alternative AMCL motion models
- Visual odometry integration
- Slip detection and explicit handling

### Computational Requirements

- AMCL with 3000 particles is CPU-intensive
- RViz + Gazebo + Nav2 requires modern hardware
- For embedded deployment, parameter reduction needed

### Sensor Limitations

- 10m LIDAR range limits performance in large open spaces
- No dynamic obstacle handling (static map assumption)
- IMU noise impacts EKF accuracy

---

## Documentation

### Main Report

**Location:** `docs/compiled/final_project_report.pdf` (after compilation)

**Contents:**
- Complete theoretical foundations (EKF, SLAM, AMCL, Nav2)
- Detailed implementation for all phases
- Comprehensive troubleshooting documentation
- Performance analysis and results
- Lessons learned and best practices

### Phase Reports (Archived)

Individual phase reports (markdown) have been consolidated into the LaTeX report. They remain in the workspace locally for reference but are not tracked in git.

---

## Development Notes

### Platform-Specific: Pop!\_OS / NVIDIA

All wrapper scripts (`run_*.sh`) configure the environment for stable GUI operation:
- Force X11 (avoid Wayland issues)
- NVIDIA GPU settings
- OpenGL/Ogre rendering configuration
- X11 connection permissions

**Without these settings:** Gazebo and RViz crash or display black screens.

### QoS Profiles Cheat Sheet

| Topic Type | QoS Reliability | QoS Durability |
|------------|-----------------|----------------|
| Sensors (/scan, /imu) | Best Effort | Volatile |
| Commands (/cmd\_vel) | Reliable | Volatile |
| Maps (/map) | Reliable | Transient Local |
| State (/odom) | Reliable | Volatile |

**Rule:** Subscribers must match publisher QoS or connection fails silently.

### Lifecycle Nodes

Nav2 and SLAM Toolbox use lifecycle management. Always use lifecycle managers with `autostart: true`:

```yaml
lifecycle_manager:
  ros__parameters:
    autostart: true
    node_names: ['map_server', 'amcl', 'controller_server', 
                 'planner_server', 'behavior_server', 'bt_navigator']
```

---

## Performance Metrics

From Phase 6 analysis:

| Metric | Value |
|--------|-------|
| Single-goal success rate | ~95% (19/20) |
| Three-waypoint patrol completion | ~75% (6/8) |
| Mean path tracking deviation | 0.15 m |
| Maximum path deviation | 0.35 m |
| RMS path deviation | 0.18 m |
| Goal position error | <0.1 m |

---

## References and Resources

### Official Documentation
- [ROS 2 Jazzy](https://docs.ros.org/en/jazzy/)
- [Nav2](https://navigation.ros.org/)
- [SLAM Toolbox](https://github.com/SteveMacenski/slam_toolbox)
- [Gazebo Harmonic](https://gazebosim.org/docs/harmonic/)

### Course Materials
- MOGI-ROS Navigation Tutorial: https://github.com/MOGI-ROS/Week-7-8-ROS2-Navigation
- Video Playlist: https://www.youtube.com/playlist?list=PLRE44FoOoKf7NzWwxt3W2taZ7BiWyfhCp

---

## Contact

**Author:** Diovandi Basheera Putra  
**Institution:** Swiss German University  
**Course:** Autonomous Mobile Robot (2025-2026/2025-1-2943)  
**Instructor:** Dr. Rusman Rusyadi

---

## License

This project is academic coursework. Please respect academic integrity guidelines if referencing this work.

---

**Last Updated:** December 18, 2025  
**Branch:** `final-project`  
**ROS 2 Version:** Jazzy Jalisco  
**Gazebo Version:** Harmonic
