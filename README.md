# Domestic Mobile Robot Project (ROS 2 Jazzy)

**Author:** Diovandi Basheera Putra  
**Course:** Autonomous Mobile Robot  
**Description:** A complete autonomous navigation stack for a custom differential drive robot in Gazebo.

## Project Structure

This repository contains the `exercise_launch` package, which orchestrates the entire project.

```
src/exercise_launch/
├── config/             # Configuration files (Nav2, SLAM, EKF, Bridge)
├── launch/             # Launch files for all phases
├── urdf/               # Robot description (Xacro)
├── worlds/             # Gazebo worlds
├── scripts/            # Helper scripts (startup, analysis)
└── exercise_launch/    # Python nodes (patrol logic, analysis)
```

## Prerequisites

- **OS:** Ubuntu 24.04 (Pop!_OS recommended for NVIDIA setups)
- **ROS 2:** Jazzy Jalisco
- **Simulator:** Gazebo Harmonic
- **Dependencies:**
  ```bash
  sudo apt install ros-jazzy-navigation2 ros-jazzy-nav2-bringup ros-jazzy-slam-toolbox \
  ros-jazzy-robot-localization ros-jazzy-ros-gz ros-jazzy-plotjuggler-ros \
  ros-jazzy-teleop-twist-keyboard ros-jazzy-topic-tools
  ```

## Build Instructions

```bash
cd ~/ros2_ws
colcon build --packages-select exercise_launch
source install/setup.bash
```

## Running the Phases

### Phase 1 & 2: Simulation & Sensors
Launch the robot in the house environment with raw sensor feeds.
```bash
./src/exercise_launch/scripts/run_phase2.sh
```

### Phase 3: SLAM (Mapping)
Generate a map of the environment. Drive the robot using the terminal teleop.
```bash
# Terminal 1: Launch SLAM
./src/exercise_launch/scripts/run_slam.sh

# Terminal 2: Save the map (when done)
./src/exercise_launch/scripts/save_map.sh
```

### Phase 4: Navigation
Launch the full Nav2 stack. You can set goals using the "2D Goal Pose" tool in RViz.
```bash
./src/exercise_launch/scripts/run_navigation.sh
```

### Phase 5: Autonomous Patrol
Run the automated patrol script (Kitchen -> Bedroom -> Home).
```bash
# Terminal 1: Launch Navigation (Headless or with RViz)
./src/exercise_launch/scripts/run_record_patrol.sh

# Terminal 2: Start the Patrol
ros2 run exercise_launch house_patrol
```

### Phase 6: Analysis
Playback recorded data and analyze performance.
```bash
# Playback the bag and visualize analysis
./src/exercise_launch/scripts/run_analyze_patrol.sh
```

## Documentation

- **Final Report**: `docs/FINAL_PROJECT_REPORT.md`
- **Phase Reports**: Detailed technical logs for each phase are available in `docs/phase*_final_project_report.md`.
