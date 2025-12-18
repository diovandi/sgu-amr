## Full Technical Report of Phase 5 & 6: Autonomous Patrol & Offline Analysis

### 0. Scope

This document is a **chronological, technical report** of everything that happened during Phase 5 (Navigation Data Collection) and Phase 6 (Playback & Analysis) of the domestic mobile robot project. It details the transition from manual navigation and mapping to full autonomy and high-fidelity offline performance analysis.

It covers:

- All major **code and configuration changes** in `exercise_launch` (patrol scripts, analysis nodes, specialized URDFs, launch files, and shell wrappers).
- All significant **bugs, misdiagnoses, and fixes** encountered during implementation.
- The evolution of the **patrol logic**, **coordinate transformation system**, and **visualization suite**.
- All **troubleshooting steps** and **error resolutions**, including:
  - Nav2 lifecycle and "BasicNavigator" synchronization.
  - RViz coordinate frame mismatches during playback.
  - The `tf2_geometry_msgs` API crash in ROS 2 Jazzy.
  - The "Detached Wheels" visualization bug and its various attempted fixes.
  - PlotJuggler live topic streaming vs. bag data loading.

---

## 1. Starting Point and Goals

### 1.1 Project Context

This work concludes the **final project** for the **Autonomous Mobile Robot** course (2025-2026/2025-1-2943) instructed by **Dr. Rusman Rusyadi** at Swiss German University.

Previous phases established:
- **Phase 1-2**: Robot URDF, simulation infrastructure, and LIDAR processing.
- **Phase 3**: SLAM implementation and house map generation.
- **Phase 4**: Navigation (Nav2) setup with EKF and AMCL.

### 1.2 Phase 5 & 6 Objectives

**Goal**: Automate house patrol and create a comprehensive analysis suite to compare planned vs. actual paths.

**Specific Requirements**:

1. **Phase 5 (Automation)**:
   - Implement a waypoint recorder that captures 3 points (Kitchen, Bedroom, Home) from RViz clicks.
   - Create an autonomous patrol script (`house_patrol.py`) using `BasicNavigator`.
   - Create a headless recording launch file (`record_patrol.launch.py`) to save system data to bags.

2. **Phase 6 (Analysis)**:
   - Implement an analysis node (`patrol_analysis.py`) to calculate actual paths taken in the `map` frame during playback.
   - Stream path data to PlotJuggler for XY comparison.
   - Overlay the actual map and robot model in RViz during bag playback.

---

## 2. Phase 5 Implementation: Automation

### 2.1 Waypoint Recorder (`patrol_waypoint_recorder.py`)

Created a dedicated node to subscribe to `/clicked_point`.

**Key Features**:
- Listens for exactly 3 clicks.
- Automatically maps clicks to "kitchen", "bedroom", and "home".
- Saves to `~/ros2_ws/config/patrol_waypoints.yaml`.

**YAML Structure**:
```yaml
waypoints:
  kitchen: {frame_id: map, x: 1.2, y: -0.5, yaw: 0.0}
  bedroom: {frame_id: map, x: -2.3, y: 1.1, yaw: 0.0}
  home: {frame_id: map, x: 0.0, y: 0.0, yaw: 0.0}
```

### 2.2 House Patrol Script (`house_patrol.py`)

Implemented the core patrol logic using `nav2_simple_commander`.

**Key Logic**:
- `navigator.waitUntilNav2Active()`: Essential to prevent script crashes if Nav2 hasn't fully booted.
- `navigator.setInitialPose()`: Sets the robot's starting belief to (0,0) in the map.
- **Task Monitoring**: Uses `navigator.isTaskComplete()` in a loop with feedback logging.
- **Robustness**: If a goal result is `FAILED` or `CANCELED`, the script logs the specific error code and continues to the next waypoint.

### 2.3 Headless Recording Launch (`record_patrol.launch.py`)

Created to facilitate clean data recording without the overhead of RViz.

**Changes to `navigation.launch.py`**:
Added launch arguments to make the GUI optional:
```python
DeclareLaunchArgument('use_rviz', default_value='true')
...
rviz_node = Node(..., condition=IfCondition(LaunchConfiguration('use_rviz')))
```

---

## 3. Phase 6 Implementation: Analysis Suite

### 3.1 Patrol Analysis Node (`patrol_analysis.py`)

This node is the "brain" of the offline analysis.

**Functionality**:
- **Transform Lookup**: Uses `tf2_ros` to find the `map → odom` transform.
- **Path Calculation**:
  - Subscribes to `/odometry/filtered` (from the bag).
  - Transforms the odom-frame pose into the map-frame.
  - Appends to a `nav_msgs/Path` published as `/actual_path`.
- **Plotting Support**: Publishes `PointStamped` messages to `/planned_xy` and `/actual_xy` for PlotJuggler streaming.

### 3.2 Analysis Launch File (`analyze_patrol.launch.py`)

Orchestrates the playback environment.

**Key Components**:
- `ros2 bag play --clock`: Uses the recorded clock for synchronization.
- `nav2_map_server`: Loads `house_map.yaml` to provide a static background.
- `nav2_lifecycle_manager`: Transitions the map server to the `active` state.
- `robot_state_publisher`: Processes the URDF for rendering.

---

## 4. Troubleshooting and Issues

### 4.1 Issue #1: `patrol_analysis` Crash (Jazzy API Mismatch)

**Symptoms**:
Node crashed immediately upon receiving the first odometry message during playback.
`AttributeError: 'PoseStamped' object has no attribute 'position'`

**Diagnosis**:
The `do_transform_pose` function in the ROS 2 Jazzy version of `tf2_geometry_msgs` was being passed a `PoseStamped` but was internally trying to access `.position` directly on that object, which only exists on the `.pose` member.

**Fix**:
Implemented a more robust transform wrapper:
```python
from tf2_geometry_msgs import do_transform
# Attempt direct transform first (handles PoseStamped in newer versions)
try:
    pose_map = do_transform(pose_odom, tf)
except Exception:
    # Fallback for older API versions
    pose_map_raw = do_transform_pose(pose_odom.pose, tf)
    # Re-wrap in PoseStamped
```

### 4.2 Issue #2: PlotJuggler Missing "Live" Topics

**Symptoms**:
User reported that `/planned_xy` and `/actual_xy` were missing from the list when loading the bag in PlotJuggler.

**Diagnosis**:
The user was attempting to "Load Data" from the recorded `.mcap` file. However, these specific analysis topics are calculated **on-the-fly** during playback and are not stored in the original bag.

**Fix**:
Instructed the user to use PlotJuggler's **Streaming** feature:
1. Start `run_analyze_patrol.sh`.
2. In PlotJuggler, go to `Streaming` -> `ROS 2 Topic Subscriber`.
3. Select the `_xy` topics to see the live comparison.

### 4.3 Issue #3: The "Detached Wheels" Visualization Bug

**Symptoms**:
In RViz, the robot's chassis was visible, but the wheels appeared at the origin (0,0) or were missing entirely, with errors: `No transform from [left_wheel] to [base_link]`.

**Attempted Fix 1 (Joint State Publisher)**:
Added `joint_state_publisher` to provide default wheel positions.
**Result**: Failed. The bag's recorded `/tf_static` or `/tf` was conflicting with the new transforms.

**Attempted Fix 2 (Ignore Timestamp)**:
Added `ignore_timestamp: True` to `robot_state_publisher`.
**Result**: Failed. Continuous joints still required data that wasn't in the bag.

**Attempted Fix 3 (Topic Remapping)**:
Remapped `/joint_states` to a private namespace `/analysis/joint_states`.
**Result**: Failed. The fundamental conflict between the bag's TF tree and the node's output persisted.

**Final Successful Fix (Specialized Analysis URDF)**:
Created `urdf/rudimentary_bot_analysis.urdf.xacro`.
**Changes**: Converted `left_wheel_joint` and `right_wheel_joint` from `continuous` to `fixed`.
**Rationale**: By making the wheels "fixed" in the URDF, they become part of the static robot geometry. RViz no longer looks for a dynamic transform for them, and they appear perfectly attached to the chassis during playback.

---

## 5. Summary of All Changes

### 5.1 New Files Created

- **`src/exercise_launch/exercise_launch/patrol_waypoint_recorder.py`**: Node for capturing RViz clicks to YAML.
- **`src/exercise_launch/exercise_launch/house_patrol.py`**: Autonomous patrol orchestration node.
- **`src/exercise_launch/exercise_launch/patrol_analysis.py`**: Transformation and path accumulation node.
- **`src/exercise_launch/launch/record_patrol.launch.py`**: Headless recording pipeline.
- **`src/exercise_launch/launch/analyze_patrol.launch.py`**: Playback and analysis pipeline.
- **`src/exercise_launch/rviz/patrol_analysis.rviz`**: Analysis-specific RViz configuration (Map + Paths + Model).
- **`src/exercise_launch/urdf/rudimentary_bot_analysis.urdf.xacro`**: Optimized URDF with fixed joints for playback.

### 5.2 Scripts Created (Wrapper Suite)
- `scripts/run_patrol_waypoint_recorder.sh`
- `scripts/run_record_patrol.sh`
- `scripts/run_analyze_patrol.sh`

### 5.3 Files Modified

- **`src/exercise_launch/launch/navigation.launch.py`**: Added `use_rviz` and `use_goal_bridge` logic for headless support.
- **`src/exercise_launch/setup.py`**: Added 3 new entry points for Phase 5 & 6 nodes.
- **`src/exercise_launch/package.xml`**: Added `nav2_simple_commander`, `plotjuggler`, and `tf2_geometry_msgs` dependencies.

---

## 6. Lessons Learned and Best Practices

### 6.1 Bag Playback and TF Conflicts
- **Static vs. Dynamic TF**: Recorded bags often contain `/tf_static`. Trying to publish the same transforms during playback using `robot_state_publisher` will cause "jitter" or "detachments" in RViz.
- **The "Fixed Joint" Trick**: For offline visualization where joint movement (like wheel rotation) isn't recorded, converting those joints to `fixed` in a temporary URDF is the most reliable way to ensure a complete robot model in RViz.

### 6.2 Nav2 API and Lifecycle
- **BasicNavigator**: It is significantly more stable than writing custom Action Clients for simple patrols, but it **must** be allowed to wait for the Nav2 servers to be fully `active` before sending goals.

### 6.3 Visualization and Performance
- **Headless Recording**: Recording sensor data while a GUI (RViz) is running can lead to dropped messages or "late" TFs in the bag due to CPU contention. Always record headless when data fidelity is a priority.

---

## 7. Final State

### 7.1 Working Components

1. **Patrol System**:
   - ✅ 3-click waypoint setup.
   - ✅ Autonomous loop with `BasicNavigator`.
   - ✅ Error handling for failed waypoints.

2. **Analysis Pipeline**:
   - ✅ Map-frame path reconstruction.
   - ✅ Planned vs. Actual path comparison (Red vs. Green) in RViz.
   - ✅ Live XY streaming to PlotJuggler.
   - ✅ Static Map background in playback.

3. **Rendering**:
   - ✅ Full robot model rendering with attached wheels using the Analysis URDF.

### 7.2 Objective Status
✅ **All objectives achieved.** The system provides a professional-grade environment for robot performance evaluation, bridging the gap between simulation and data-driven analysis.

---

**End of Phase 5 & 6 Report**
