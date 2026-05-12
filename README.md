# SGU Autonomous Mobile Robot Workspace

**Author:** Diovandi Basheera Putra  
**Course:** Autonomous Mobile Robot, Swiss German University
**ROS 2:** Jazzy Jalisco
**Simulator:** Gazebo Harmonic

This repository is a ROS 2 colcon workspace containing two related coursework tracks:

- **Exercises 3 & 4:** TurtleBot 4 pillar following, rosbag recording, EKF playback, and plotting.
- **Final Project:** A domestic mobile robot simulation with SLAM, Nav2 navigation, autonomous patrol, and report material.

The shared ROS package is `src/exercise_launch`. Package paths stay stable so the workspace can still be built and launched with standard ROS 2 tooling.

## Repository Layout

```text
.
├── src/exercise_launch/          # ROS 2 package: launch files, nodes, configs, worlds, RViz, URDF
├── data/final-project/           # Tracked final-project map and patrol metadata
├── docs/exercises/               # Exercise 3/4 screenshots and plotted evidence
├── docs/final-project/           # Final-project README, reports, LaTeX source, bibliography
└── exercise4_recording/          # Local rosbag output location; large bag payloads are ignored
```

## Requirements

- Ubuntu 24.04 or Pop!_OS 24.04
- ROS 2 Jazzy
- Gazebo Harmonic
- Common packages:

```bash
sudo apt install ros-jazzy-navigation2 ros-jazzy-nav2-bringup ros-jazzy-slam-toolbox \
  ros-jazzy-robot-localization ros-jazzy-ros-gz ros-jazzy-plotjuggler-ros \
  ros-jazzy-teleop-twist-keyboard ros-jazzy-topic-tools
```

## Build

```bash
cd ~/ros2_ws
source /opt/ros/jazzy/setup.bash
colcon build --packages-select exercise_launch
source install/setup.bash
```

The helper scripts under `src/exercise_launch/scripts/` resolve the workspace root automatically and export `SGU_AMR_WORKSPACE`. If you launch nodes manually from outside the workspace root, set it yourself:

```bash
export SGU_AMR_WORKSPACE=/path/to/sgu-amr
```

## Exercises 3 & 4

Exercise 3 runs TurtleBot 4 pillar following in Gazebo:

```bash
./src/exercise_launch/scripts/run_exercise3.sh
```

Exercise 4 records and replays bag data with EKF filtering:

```bash
./src/exercise_launch/scripts/run_exercise4_record.sh
./src/exercise_launch/scripts/run_exercise4.sh
```

Screenshots and the Exercise 4 path plot are tracked in `docs/exercises/`.

## Final Project

The final project uses the custom `rudimentary_bot` in a domestic environment with SLAM, AMCL/Nav2, waypoint patrol, and offline analysis.

```bash
# Phase 1/2: simulation and sensor visualization
./src/exercise_launch/scripts/run_phase2.sh

# Phase 3: SLAM mapping
./src/exercise_launch/scripts/run_slam.sh
./src/exercise_launch/scripts/save_map.sh

# Phase 4: Nav2 navigation
./src/exercise_launch/scripts/run_navigation.sh

# Phase 5: waypoint patrol recording
./src/exercise_launch/scripts/run_patrol_waypoint_recorder.sh
./src/exercise_launch/scripts/run_record_patrol.sh

# Phase 6: playback and analysis
./src/exercise_launch/scripts/run_analyze_patrol.sh
```

Default final-project runtime data lives in:

- `data/final-project/maps/house_map.yaml`
- `data/final-project/maps/house_map.pgm`
- `data/final-project/patrol/patrol_waypoints.yaml`
- `data/final-project/patrol/metadata.yaml`

More details are in `docs/final-project/README.md`. The final report sources and build guide are also under `docs/final-project/`.
