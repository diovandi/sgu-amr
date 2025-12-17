## ROS 2 Exercises 3 & 4 – TurtleBot 4 (UCR EE106)

This workspace contains my ROS 2 Jazzy implementation of **Exercise 3 (pillar following)** and **Exercise 4 (bagging + EKF + plotting)** from UCR EE106, adapted to the **TurtleBot 4** in **Gazebo Sim** with **RViz2**.  
All code lives under the `src/` directory, primarily in the `exercise_launch` package.

The full lab report with technical details is in `ros2_exercises3_4_report.md`.  
Once published, the GitHub repository URL for this workspace will be added here, and referenced from the report.

---

### 1. Requirements

- **OS**: Linux (tested on Pop!_OS 24.04 with NVIDIA GPU)
- **ROS 2**: Jazzy Jalisco (`/opt/ros/jazzy`)
- **Simulation stack**:
  - `turtlebot4_gz_bringup`
  - `turtlebot4_description`
- **Visualization / tools**:
  - `rviz2`
  - `teleop_twist_keyboard`
  - `robot_localization`
  - Plotting (for Exercise 4):
    - `ros-jazzy-plotjuggler-ros` (PlotJuggler + ROS 2 plugin)

Install PlotJuggler on Jazzy, for example:

```bash
sudo apt update
sudo apt install ros-jazzy-plotjuggler-ros
```

---

### 2. Workspace Layout

This repository is a **colcon workspace**:

- `src/exercise_launch/`
  - `launch/`: `exercise1.launch.py`, `exercise3.launch.py`, `exercise4_record.launch.py`, `exercise4.launch.py`
  - `scripts/`: `run_exercise3.sh`, `run_exercise4_record.sh`, `run_exercise4.sh`
  - `config/`: `ekf_params.yaml`, `rqt_multiplot.xml` (legacy)
  - `models/`: `pillar/model.sdf` (cylindrical pillar)
  - `rviz/`: `exercise3.rviz`, `exercise4.rviz`
  - `exercise_launch/`: Python code (`pillar_driver.py`)
  - `test/`: ament lint tests
- `exercise4_recording/`: recorded MCAP bag (optional to track in git)
- `ros2_exercises3_4_report.md`: course report for Exercises 3 & 4
- Standard colcon output directories (created after build): `build/`, `install/`, `log/`

---

### 3. Building the Workspace

Clone this repository (or copy the workspace) and build with colcon:

```bash
cd ~/ros2_ws        # adjust if you use a different path
source /opt/ros/jazzy/setup.bash

colcon build --packages-select exercise_launch
```

Source the overlay before running anything:

```bash
source /opt/ros/jazzy/setup.bash
source ~/ros2_ws/install/setup.bash
```

---

### 4. Exercise 3 – Real-Time Pillar Following

Exercise 3 launches the TurtleBot 4 in the **warehouse world**, spawns a red pillar model in front of the robot, and runs a custom `pillar_driver` node that drives toward the pillar while avoiding the dock.

#### 4.1 Launching Exercise 3

Use the helper script (recommended on NVIDIA/Wayland systems):

```bash
cd ~/ros2_ws
./src/exercise_launch/scripts/run_exercise3.sh
```

This script:

- Sets X11/OpenGL/NVIDIA environment variables for stable Gazebo + RViz2.
- Sources `/opt/ros/jazzy` and `~/ros2_ws/install`.
- Launches `exercise3.launch.py`, which:
  - Starts `turtlebot4_gz_bringup` (warehouse world) and spawns TurtleBot 4.
  - Spawns the pillar via `ros_gz_sim create`.
  - Starts RViz2 with `exercise3.rviz`.
  - Starts `pillar_driver_node` after a short delay.

#### 4.2 Controller Behaviour (Summary)

The `pillar_driver` node:

- Subscribes to `/scan` (BEST_EFFORT QoS) and `/odom`.
- Publishes `geometry_msgs/TwistStamped` on `/cmd_vel`.
- Implements:
  - Startup **wait** then **straight back‑off** from the dock.
  - A **forward search cone** and **obstacle cone** in front of the robot.
  - A pillar detector that clusters LiDAR points and selects clusters whose width and range match the cylindrical pillar, to avoid locking onto the dock or the human model.
  - An **odom‑based external back‑off detector** that notices when the Create 3 `motion_control` stack is backing the robot away while the node is commanding forward.
  - A multi‑stage **recovery FSM** (pause → back → 45° left turn → forward‑clear) after collisions.
  - A **mission‑complete** latch once the robot reaches the desired distance (~0.5 m) from the pillar.

---

### 5. Exercise 4 – Bag Recording, EKF and Plotting

Exercise 4 records the relevant topics while the robot is driven, then replays the rosbag and runs an EKF (`robot_localization`) to produce `/odometry/filtered`, which is visualized in RViz2 and plotted with PlotJuggler.

#### 5.1 Recording a Bag

Launch the recording setup:

```bash
cd ~/ros2_ws
./src/exercise_launch/scripts/run_exercise4_record.sh
```

This:

- Starts the same TurtleBot 4 simulation (warehouse world + pillar).
- Opens `teleop_twist_keyboard` (stamped) in `xterm` for manual driving.
- Runs `ros2 bag record` to log:

  - `/odom`, `/imu`, `/scan`, `/tf`, `/tf_static`,
  - `/robot_description`, `/joint_states`.

Drive a trajectory with rotations and straight segments, then stop with `Ctrl+C`.  
The bag is saved under `exercise4_recording/` (e.g., `exercise4_recording_0.mcap`).

#### 5.2 Playback with EKF and RViz

Play back the bag and run EKF:

```bash
cd ~/ros2_ws
./src/exercise_launch/scripts/run_exercise4.sh
# internally: ros2 launch exercise_launch exercise4.launch.py bag_path:=exercise4_recording_0.mcap
```

`exercise4.launch.py`:

- Runs `ros2 bag play <bag> --clock --rate 1.0` (the **bag owns `/tf` and `/tf_static`**).
- Starts `robot_state_publisher` from the TurtleBot 4 URDF.
- Starts `ekf_node` from `robot_localization` using `config/ekf_params.yaml`:
  - `publish_tf: false` – EKF outputs `/odometry/filtered` only, no TF.
- Launches RViz2 with `exercise4.rviz`, which shows:

  - Raw `/odom` vs `/odometry/filtered` (different colours).
  - Robot model and TF tree.
  - `LaserScan` on `/scan`.

#### 5.3 Plotting with PlotJuggler

To reproduce the Exercise 4 trajectory plots:

1. Start bag playback as above (or skip to step 3 to load offline).
2. In another terminal:

   ```bash
   source /opt/ros/jazzy/setup.bash
   source ~/ros2_ws/install/setup.bash
   plotjuggler
   ```

3. Either:

   - Use **Streaming → ROS2 Topic Subscriber → Start**, then create an XY curve with:
     - X = `/odometry/filtered/pose/pose/position/x`
     - Y = `/odometry/filtered/pose/pose/position/y`,  
     and similarly for `/odom`; or
   - Use **File → Data → Load from rosbag2 (ROS 2)** and select `exercise4_recording`.

4. Use the auto‑scale button to see the full path and compare EKF vs raw odometry.

---

### 6. Git Usage and Repository Notes

This entire `ros2_ws` workspace (excluding large bag files and personal documents) is intended to be pushed to GitHub as a teaching artifact for Exercises 3 and 4.

Suggested steps (once `.gitignore` is in place):

```bash
cd ~/ros2_ws
git init
git add .
# The .gitignore in this workspace excludes colcon build artifacts,
# MCAP bag files under exercise4_recording/, PDFs, and long-form notes.
git commit -m "Initial commit: ROS 2 Exercises 3 & 4 for TurtleBot 4"
git branch -M main
git remote add origin git@github.com:<your-user>/<your-repo>.git
git push -u origin main
```

After the repository exists, add its URL here (e.g. `Repository: https://github.com/...`) and optionally reference it in `ros2_exercises3_4_report.md` so your professor can clone the exact workspace you used.  
The report file itself is not tracked in the repository; only the code, configuration, and this `README.md` are shared.

