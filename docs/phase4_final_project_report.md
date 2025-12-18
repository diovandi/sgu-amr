## Phase 4 Final Project Report — Nav2 Configuration (Domestic Mobile Robot)

### 0. Document scope, evidence rules, and how to read this report

This document is the **Phase 4 technical report** for the Domestic Mobile Robot project: configuring, integrating, and debugging the ROS 2 Navigation Stack (Nav2) so the custom differential-drive robot (`rudimentary_bot`) can **localize on a saved map** and **navigate to operator-set goals** inside the Gazebo “home” world.

This report is written to match the **style and depth** of your Phase 1–3 final reports: it is long-form, chronological, and explicitly captures the iterative debugging process (including mistakes and reversions).

#### 0.1 Evidence sources used in this report

This report is constructed from **two** types of evidence:

- **Workspace state (ground truth)**: current files in `~/ros2_ws/src/exercise_launch/` and their exact contents.
- **Runtime log evidence**: the captured terminal output in `/home/dio/.cursor/projects/home-dio-ros2-ws/terminals/17.txt` (one complete run containing multiple goal attempts, planner/controller messages, and shutdown output).

Additionally, where earlier iterations are not present in `17.txt`, those steps are recorded as:

- **User-reported session history** (verbatim symptoms and parameter snapshots from the session summary you provided in chat). These are clearly labeled as “User-reported”.

#### 0.2 Why the “no mistakes” requirement is hard and how this report enforces it

Some Phase 4 issues occurred in earlier runs whose terminal logs are not available in the workspace (only `17.txt` exists). To avoid inventing details:

- Anything that is **not directly present** in `17.txt` or in the current config files is written as **User-reported** and/or **reconstructed from the provided chat summary**, not as an observed log fact.
- Parameter values in the **final configuration** are always taken from the current files (not from memory).

#### 0.3 Phase 4 definition (what counts as “done”)

Phase 4 is considered successful when all of the following are true:

- Gazebo simulation launches reliably with the robot model, sensor streams, and `/clock`.
- The TF tree is valid for navigation: **`map → odom → base_link → (laser_link, imu_link, wheels…)`**.
- Nav2 nodes launch cleanly (or with only non-fatal warnings) and become “active” via lifecycle.
- RViz shows the map, robot model, scan, and navigation outputs.
- An operator can set a goal in RViz and the robot drives toward it.
- Localization remains stable enough to execute multiple commands without the scan/map “tearing away”.

---

## 1. Starting point at Phase 4 entry

### 1.1 What Phase 3 delivered to Phase 4

By Phase 3, the project already had:

- A Gazebo world of a house environment: `src/exercise_launch/worlds/home.sdf`.
- The custom robot URDF/Xacro:
  - `src/exercise_launch/urdf/rudimentary_bot.urdf.xacro`
- Simulation launch infrastructure:
  - `src/exercise_launch/launch/simulation.launch.py` (spawns robot, sets up the Gazebo bridge, robot state publisher, etc.)
- EKF configuration for state estimation:
  - `src/exercise_launch/config/ekf_slam_params.yaml`
- A saved map produced during Phase 3 SLAM:
  - `~/ros2_ws/maps/house_map.yaml` (referenced later by Phase 4 launch)

Phase 4 builds on this by:

- Replacing “SLAM” with “localization on an existing map” (AMCL + map_server)
- Introducing the Nav2 planning/control pipeline

### 1.2 Phase 4 prompt requirements (explicit)

The Phase 4 prompt required:

1) Create a Nav2 params file `nav2_params.yaml` for a differential drive robot and include these critical tuning values:

- `robot_radius: 0.25`
- `inflation_radius: 0.45` (door clearance for 0.9m doors)
- `cost_scaling_factor: 5.0`
- `use_sim_time: true`
- controller must be **RegulatedPurePursuitController**

2) Create a launch file `navigation.launch.py` that:

- accepts arguments: `map` (path to YAML) and `use_sim_time`
- includes `simulation.launch.py`
- runs `robot_localization` EKF
- includes `nav2_bringup/bringup_launch.py` and passes the map and nav2 params

3) Operator workflow additions requested after initial implementation:

- RViz should launch alongside Gazebo.
- The operator should be able to click a point in RViz and have the robot navigate there.

---

## 2. Final architecture (what Phase 4 ended up launching)

Phase 4 ended up being a composed pipeline:

- Gazebo (gz-sim)
- ros_gz_bridge parameter bridge:
  - `/cmd_vel` ROS → Gazebo
  - `/odom`, `/scan`, `/imu`, `/clock` Gazebo → ROS
- robot_state_publisher and joint_state_publisher
- robot_localization EKF:
  - fuses `/odom` + `/imu`
  - publishes `odom → base_link` TF
- Nav2 bringup:
  - `map_server` publishes `/map`
  - `amcl` publishes `map → odom`
  - `planner_server` creates global plans
  - `controller_server` executes plans (Regulated Pure Pursuit)
  - costmaps track obstacles and inflation
  - lifecycle managers bring nodes up in a consistent order
- RViz:
  - visualizes map, TF, robot, scan, and plan
- A custom goal bridge:
  - converts RViz “2D Goal Pose” topic (`/goal_pose`) into the Nav2 NavigateToPose action

The final topology is what you want for a domestic mobile robot: **AMCL for localization** + **EKF for odom smoothing** + **Nav2 for navigation**.

---

## 3. Files created/modified for Phase 4 (final state)

### 3.1 Core Phase 4 files

- `src/exercise_launch/config/nav2_params.yaml`
  - Full Nav2 configuration: AMCL, controller, planner, costmaps, collision monitor, smoother, etc.

- `src/exercise_launch/launch/navigation.launch.py`
  - One command launches simulation + EKF + Nav2 + RViz + goal bridge

- `src/exercise_launch/rviz/navigation.rviz`
  - RViz preset tailored for Nav2 (fixed frame, map QoS, scan display, tools)

- `src/exercise_launch/exercise_launch/nav2_goal_bridge.py`
  - New node to turn RViz goal messages into Nav2 action goals

- `src/exercise_launch/scripts/run_navigation.sh`
  - Wrapper script to run the launch reliably on Pop!_OS/NVIDIA/XWayland setups

### 3.2 Phase 4-critical dependencies on earlier-phase files

- `src/exercise_launch/launch/simulation.launch.py`
  - Supports a `use_ekf_tf` argument; Phase 4 passes `true` to avoid the Phase 2 static TF.

- `src/exercise_launch/config/ekf_slam_params.yaml`
  - EKF fuses `/odom` + `/imu`, publishes `odom → base_link`

- `src/exercise_launch/urdf/rudimentary_bot.urdf.xacro`
  - The physical lidar range (`<range><max>…</max>`) is a key Phase 4 localization constraint.

---

## 4. Phase 4 implementation (baseline build-out)

This section describes what was implemented first before the heavy troubleshooting.

### 4.1 Baseline `nav2_params.yaml` generation

A “standard Nav2 params” file was created for a differential drive robot, then iterated toward:

- correct frame IDs for this robot (`base_link`, `odom`, `map`)
- correct observation sources (`scan`)
- required costmap sizing and inflation
- controller plugin swap to Regulated Pure Pursuit

A Nav2 default file was also kept as a reference baseline:

- `src/exercise_launch/config/nav2_params.yaml.backup_default`

That file still contains the upstream-default patterns (e.g., `base_footprint`, MPPI controller) and is used later in this report as the “before” reference.

### 4.2 Baseline `navigation.launch.py` generation

The launch file was implemented to do the following:

- Declare launch args:
  - `map`
  - `use_sim_time`
  - `rviz_config`
- Include `simulation.launch.py` (spawns Gazebo and robot)
- Run EKF
- Include Nav2 bringup
- Run RViz

Important: in the final state, some launch arguments are declared but not correctly plumbed into includes (this is documented as a known-spec mismatch in §12.3).

### 4.3 RViz integration + click-to-goal integration

- RViz was added to the navigation launch.
- A dedicated RViz config (`navigation.rviz`) was maintained.
- A custom node (`nav2_goal_bridge`) was created and installed via `setup.py` entry points.

---

## 5. Major troubleshooting timeline (chronological)

Phase 4 was not a linear “write files and done” phase. It consisted of multiple waves:

1) **Get Nav2 to launch** without YAML type errors.
2) **Get RViz to display map and TF** (map QoS + correct fixed frame + AMCL producing map→odom).
3) **Get robot motion** (fix base frame mismatches that prevent control).
4) **Fix localization stability / desynchronization** (AMCL tuning + physical lidar constraints + TF authority issues).
5) **Attempt optimizations** and revert anything that makes behavior worse.

The sections below record those waves.

---

## 6. Wave 1 — Nav2 would not start (the recurring “tuple/type error”)

### 6.1 Symptom (User-reported)

Early in Phase 4, bringing up Nav2 repeatedly failed with runtime type errors described as a “tuple error”, usually reported in the form:

- `TypeError: Expected 'value' to be one of [<class 'float'>, ..., <class 'tuple'>]`

This blocked all progress because:

- lifecycle nodes never activated
- `map_server`/`amcl` did not stabilize
- RViz had no map frame

### 6.2 Root causes (User-reported + consistent with Nav2 YAML failure modes)

Nav2 parameter YAML files are “brittle” because:

- many parameters are lists of strings or lists of plugin names
- some parameters must be exactly the correct type (e.g., `observation_sources`)
- plugin nesting errors or invalid keys can crash node initialization

During this phase, several categories of misconfiguration were encountered (as summarized in the session history you provided):

- YAML syntax/type mismatches
- outdated Nav2 sections (e.g., `recoveries_server` on older versions) used instead of `behavior_server`
- incorrectly structured `observation_sources`

### 6.3 Fix strategy (User-reported)

The key corrective approach was:

- Start from a known-good upstream Nav2 template
- Reapply only the necessary deltas (frames, controller plugin, radii, inflation)

This is the typical “safe” approach for Nav2 configuration: incremental changes to a validated baseline.

### 6.4 Result

Once the parameter file stabilized, Nav2 could start consistently and lifecycle managers could reach “Managed nodes are active” states.

You can see this “healthy activation” pattern in `17.txt`:

- AMCL receives initial pose and activates:
  - `[amcl]: Activating` and `Setting pose (0.000000): 0.000 0.000 0.000`
- Navigation lifecycle manager brings nodes up:
  - `Managed nodes are active`

(These appear in the early section of `17.txt`, timestamp `1766005740–1766005750`.)

---

## 7. Wave 2 — RViz: “map frame doesn’t exist”, “no map received”, “no transforms”

### 7.1 Symptom (User-reported)

After an initial attempt to add RViz:

- RViz global error: `Frame [map] does not exist`
- Map display warning: `No map received`
- RobotModel errors: no transforms to map

You also reported:

- “map frame never became available”
- intermittent states where “the map loaded for once, but the car was outside the map”

### 7.2 Core diagnostic model

In Nav2 localization mode, the `map` frame typically becomes meaningful only when:

- `map_server` is publishing `/map`
- `amcl` is publishing `map → odom`
- `odom → base_link` exists (from EKF or another odometry source)

If **any** of these are missing or delayed, RViz fixed frame = `map` will error.

### 7.3 The QoS mismatch problem (critical)

The map topic in Nav2 uses transient-local durability.

If RViz subscribes with default volatile durability, RViz may never receive the map even though `map_server` is publishing.

### 7.4 Fix: RViz map durability set to Transient Local (workspace-evidenced)

The final RViz config explicitly sets:

- `/map` topic durability: **Transient Local**
- `/map_updates` durability: **Transient Local**

This is visible in `src/exercise_launch/rviz/navigation.rviz`:

- `Durability Policy: Transient Local` for `/map`
- `Update Topic: … Transient Local` for `/map_updates`

This resolves the “No map received” symptom.

### 7.5 Fix: enforce deterministic initial pose (workspace-evidenced)

The final `nav2_params.yaml` sets:

- `amcl.set_initial_pose: true`
- `amcl.initial_pose: {x: 0.0, y: 0.0, yaw: 0.0}`

This makes AMCL publish `map → odom` immediately upon activation.

You can see AMCL acknowledging it in `17.txt`:

- `[amcl]: initialPoseReceived`
- `[amcl]: Setting pose (0.000000): 0.000 0.000 0.000`

This reduces the chance RViz starts “too early” and never gets `map` transforms.

---

## 8. Wave 3 — “Robot plans but won’t move”

### 8.1 Symptom (User-reported)

At one point you reached a state where:

- Nav2 produced a path (“it is creating the path though”)
- The robot did not physically move

### 8.2 Root cause class: base frame mismatch

A common Nav2 failure mode is using `base_footprint` in configuration when your robot publishes `base_link` (or vice versa). This can block:

- costmap transforms
- collision monitor transforms
- controller transforms

### 8.3 Evidence: default template uses `base_footprint`

The saved upstream-like config `nav2_params.yaml.backup_default` uses `base_footprint` in at least:

- `amcl.base_frame_id: "base_footprint"`
- `collision_monitor.base_frame_id: "base_footprint"`
- `loopback_simulator.base_frame_id: "base_footprint"`

### 8.4 Fix: align everything to `base_link` (workspace-evidenced)

The final `nav2_params.yaml` uses:

- `amcl.base_frame_id: "base_link"`
- `collision_monitor.base_frame_id: "base_link"`
- costmaps `robot_base_frame: base_link`

This eliminates that entire class of transform lookup failure.

### 8.5 Result

Once base-frame IDs were consistent, the controller could publish velocity and the bridge could pass commands into Gazebo.

In `17.txt`, you can see the bridge explicitly passing Twist:

- `[gz_bridge]: Passing message from ROS geometry_msgs/msg/Twist to Gazebo gz.msgs.Twist`

And you can see multiple goal successes:

- `[controller_server]: Reached the goal!`
- `[bt_navigator]: Goal succeeded`
- `[nav2_goal_bridge]: ✓ Navigation goal succeeded!`

---

## 9. Wave 4 — The hard problem: scan/map desynchronization (localization instability)

This was the primary long-running issue in Phase 4.

### 9.1 Symptom pattern (User-reported)

You reported recurring behavior:

- Navigation works initially, but:
  - scan drifts off the map
  - alignment breaks after collisions
  - the robot “desyncs after 1 command again”
  - it can succeed once, then fail on longer traversals (“other side of the house”)

You identified a likely physical cause:

- wheel slip (especially after collisions)

### 9.2 What “desync” means in Nav2 localization terms

In AMCL localization mode, the robot’s estimated pose is produced by:

- `odom → base_link` (EKF fused odometry)
- `map → odom` (AMCL correction using scan vs map)

“Desync” happens when:

- odom diverges due to wheel slip
- AMCL cannot re-correct robustly due to insufficient features / poor sensor data / poor tuning
- TF timing issues cause scans to be dropped or processed late

### 9.3 TF timing and message filter evidence (log-evidenced)

Even in the (mostly working) `17.txt` run, early startup shows heavy message-filter stress:

- AMCL dropping scans due to queue and transform cache timing:
  - `Message Filter dropping message: frame 'laser_link' … queue is full`
  - `… timestamp on the message is earlier than all the data in the transform cache`

- Global costmap transform timing errors:
  - `Lookup would require extrapolation into the past`

This indicates that timing/TF availability was a real pressure point.

### 9.4 The startup TF gap (log-evidenced)

In `navigation.launch.py`, EKF is started with a **6.0s delay**:

- `TimerAction(period=6.0, … ekf_node …)`

But `simulation.launch.py` is invoked with `use_ekf_tf=true`, so the Phase 2 static TF (`odom → base_link`) is disabled.

That creates a startup window where `odom` frame may not exist yet.

`17.txt` shows exactly that:

- `Timed out waiting for transform from base_link to odom … Invalid frame ID "odom" … frame does not exist`

This appears at timestamp `1766005743.631826092` and repeats until EKF begins.

This startup effect does not necessarily break the final run (Nav2 eventually becomes active), but it explains:

- early costmap warnings
- early message filter drops

### 9.5 EKF compute load evidence (log-evidenced)

`17.txt` also shows the EKF reporting it cannot meet its rate:

- `Failed to meet update rate! Took 0.040…`
- `Failed to meet update rate! Took 0.096…`

This is important because:

- EKF latency impacts availability of `odom → base_link` for AMCL and costmaps
- timing jitter contributes to message filter drops and extrapolation errors

This became the basis for attempting a frequency optimization later (§11).

---

## 10. Wave 4 continued — AMCL tuning iterations (chronological)

This subsection records the AMCL tuning journey.

### 10.1 User-reported “too unstable” tuning and revert request

During Phase 4 you explicitly said:

- “its way more unstable now, i think the gains are too high. go back to the default. tune it tighter”

Interpretation (Nav2 AMCL context):

- Overly permissive motion model noise can cause particle filter instability.
- Overly aggressive sensor model confidence (or aggressive update thresholds) can cause twitching.

### 10.2 Motion model parameters (`alpha1..alpha5`)

User-reported iteration history:

- α parameters were tried at higher values (e.g., 0.5)
- then tightened down again

Final file state (workspace-evidenced):

- `alpha1..alpha5: 0.2`

This is consistent with “increase odometry effect”: smaller alphas treat motion as less noisy and effectively let odometry carry more weight.

### 10.3 Update thresholds (`update_min_d`, `update_min_a`)

User-reported issue:

- instability after short motion

Final state (workspace-evidenced):

- `update_min_d: 0.1`
- `update_min_a: 0.1`

This makes AMCL update more frequently for smaller motions, which can improve correction during drift at the cost of compute.

### 10.4 Particle counts

User-reported tuning:

- particle count was increased for robustness

Final state:

- `min_particles: 1000`
- `max_particles: 3000`

This is a “heavy” AMCL setting; it improves robustness but can increase CPU load, which can feed back into timing issues.

### 10.5 Beam handling

User-reported:

- beams were reduced/changed across iterations

Final state:

- `max_beams: 60`
- `do_beamskip: false`

This balances compute against scan fidelity.

### 10.6 Laser model and likelihood max distance

Final state:

- `laser_model_type: likelihood_field`
- `laser_likelihood_max_dist: 2.0`

This is important in indoor maps: the likelihood-field model uses obstacle distance transform; `laser_likelihood_max_dist` affects how far from obstacles the model considers measurements meaningful.

### 10.7 Laser range consistency

Final state:

- `laser_min_range: 0.4`
- `laser_max_range: 10.0`

This was kept consistent with the URDF sensor configuration (see §11).

---

## 11. The critical breakthrough — physical Lidar range in URDF, not Nav2

### 11.1 The user’s key realization (User-reported)

You stated:

- “isnt this just the nav2 params, but the robot itself may not have more than 3.5m lidar range. isnt that setting somewhere else?”

This was the most important diagnostic step of Phase 4.

### 11.2 Why this matters

Nav2/AMCL parameters can only:

- discard readings
- limit max considered range
- shape the sensor model

They **cannot** extend the physical sensor range.

If the simulator’s lidar plugin max range is small, AMCL may not see enough long-range walls to anchor localization in larger connected spaces.

### 11.3 Workspace evidence: URDF lidar `<range><max>` is 10.0

In `src/exercise_launch/urdf/rudimentary_bot.urdf.xacro`, the lidar plugin is:

- type: `gpu_lidar`
- topic: `scan`
- range:
  - `<min>0.4</min>`
  - `<max>10.0</max>`

This is the physical/simulation limit. If it were `3.5`, the scan would never contain returns beyond 3.5m.

### 11.4 Result (User-reported)

After reverting to a previously working state and aligning the sensor range correctly, you reported:

- “okay, that works. its quite stable now actually.”

This aligns with the expected effect: more distant features improve AMCL’s ability to keep global alignment across multiple rooms.

---

## 12. TF authority experiments and the “robot won’t move / won’t load” regression

### 12.1 What happened (User-reported)

During stabilization attempts you reported:

- “whatever you did, it made the robot no longer move”
- “bruh now the robot wont even load, honestly just go back to the one where it successfully did 2 commands”

### 12.2 Root cause class (User-reported)

The session history identifies a key mistake:

- Attempting to make Gazebo diff-drive publish `odom → base_link` TF by setting `<publish_odom_tf>true</publish_odom_tf>`
- Disabling EKF TF publication (`publish_tf: false`) to avoid a TF conflict

This is risky because:

- If the TF is not actually present in ROS (bridge mismatch or wrong publisher), Nav2 cannot operate.
- If both sources publish, TF conflicts also break Nav2.

### 12.3 Final design decision (workspace-evidenced)

The final state is coherent and conservative:

- In URDF:
  - `<publish_odom_tf>false</publish_odom_tf>`

- In EKF:
  - `publish_tf: true` (see `config/ekf_slam_params.yaml`)

This sets the EKF as the **single authority** for `odom → base_link` in ROS.

---

## 13. EKF frequency “optimization” attempt and why it was reverted

### 13.1 Motivation

Given the timing symptoms (message filter dropping, extrapolation errors, EKF missing its rate), a reasonable hypothesis was:

- increasing EKF frequency might reduce transform latency and improve alignment under motion

### 13.2 Action taken (User-reported)

You requested:

- “yes, apply that optimization”

The optimization described in the session history was:

- EKF `frequency` increased from 5Hz to 30Hz
- EKF `sensor_timeout` reduced

### 13.3 Outcome (User-reported)

You then reverted it:

- “its actually less stable with the freq up and hence i have reverted it.”

### 13.4 Final state (workspace-evidenced)

`src/exercise_launch/config/ekf_slam_params.yaml` currently contains:

- `frequency: 5.0`
- `sensor_timeout: 0.2`
- `publish_tf: true`

This is the stable configuration you settled on.

---

## 14. Evidence-based walkthrough of the “working” run (terminal 17)

This section ties Phase 4 behavior to actual log evidence.

### 14.1 Nav2 lifecycle activation proceeds correctly

From `17.txt`:

- AMCL activates and receives initial pose:
  - `initialPoseReceived`
  - `Setting pose … 0.000 0.000 0.000`

- Lifecycle manager reports managed nodes are active:
  - `Managed nodes are active`

### 14.2 Startup TF gap and resulting warnings

From `17.txt` early on:

- local costmap cannot transform to `odom` because `odom` frame does not exist yet:

  - `Timed out waiting for transform from base_link to odom … Invalid frame ID "odom" … frame does not exist`

This corresponds to the launch behavior:

- EKF starts after 6 seconds
- no static `odom → base_link` is published (because `use_ekf_tf=true`)

### 14.3 AMCL message filter drops at startup

From `17.txt`:

- `Message Filter dropping message … queue is full`
- `… timestamp on the message is earlier than all the data in the transform cache`

This indicates:

- transforms were not available quickly enough
- or scan frequency / CPU load exceeded message filter capacity

### 14.4 Goals succeed multiple times

From `17.txt`:

- Goal 1 set and succeeded:
  - RViz “Setting goal pose …”
  - `controller_server: Reached the goal!`
  - `bt_navigator: Goal succeeded`
  - `nav2_goal_bridge: ✓ Navigation goal succeeded!`

- Goal 2 also succeeded with the same pattern.

This confirms:

- RViz click tool works
- goal bridge works
- Nav2 planning/control works
- transforms are stable enough for multiple goals

### 14.5 Planner failure when pushing far goals

`17.txt` later shows repeated planner failures:

- `GridBased plugin failed to plan … Failed to create plan with tolerance of: 0.500000`

This is a typical Nav2 outcome when:

- the goal is outside the map bounds
- or the start/goal position is in lethal/inflated space
- or localization drift places the robot “inside” obstacles

Because this occurred after multiple successes, it is interpreted as:

- not a “stack won’t run” failure,
- but a combination of goal feasibility + local map/costmap state.

### 14.6 Controller collision/patience behavior

`17.txt` shows:

- `RegulatedPurePursuitController detected collision ahead!`
- followed by
- `Controller patience exceeded`

This is expected when:

- the robot is commanded into tight areas
- local obstacles are too close
- or the controller cannot make progress due to collision constraints

It also correlates with your earlier real-world observation:

- collisions and wheel slip are a major destabilizer.

---

## 15. Final configuration snapshot (as shipped in the workspace)

This section lists the final “stable” config values that satisfy the Phase 4 prompt.

### 15.1 Nav2 critical tuning requirements (explicitly satisfied)

From `src/exercise_launch/config/nav2_params.yaml`:

- **use_sim_time**: set in
  - `map_server`
  - `amcl`
  - `controller_server`
  - costmaps

- **robot_radius: 0.25**:
  - `local_costmap.local_costmap.ros__parameters.robot_radius: 0.25`
  - `global_costmap.global_costmap.ros__parameters.robot_radius: 0.25`

- **inflation_radius: 0.45**:
  - local + global inflation layers

- **cost_scaling_factor: 5.0**:
  - local + global inflation layers

- **Controller: RegulatedPurePursuitController**:
  - `controller_server.FollowPath.plugin: nav2_regulated_pure_pursuit_controller::RegulatedPurePursuitController`

### 15.2 AMCL (final)

Key values:

- `alpha1..alpha5: 0.2`
- `do_beamskip: false`
- `max_beams: 60`
- `min_particles: 1000`, `max_particles: 3000`
- `laser_min_range: 0.4`, `laser_max_range: 10.0`
- `laser_likelihood_max_dist: 2.0`
- `set_initial_pose: true`

### 15.3 URDF lidar (final)

Key values:

- `<update_rate>10</update_rate>` (explicitly documented as reduced to avoid queue overflow)
- `<min>0.4</min>`
- `<max>10.0</max>`

### 15.4 Launch orchestration (final)

`src/exercise_launch/launch/navigation.launch.py` does:

- includes `simulation.launch.py` with `use_ekf_tf=true`
- starts a scan throttle:
  - `topic_tools/throttle … /scan → /scan_throttled at 5Hz`
- starts EKF after 6 seconds
- starts Nav2 bringup immediately
- starts goal bridge after 12 seconds
- starts RViz

---

## 16. Mistakes, regressions, and explicit lessons learned

This section is intentionally blunt.

### 16.1 Mistake: assuming “laser_max_range” in AMCL changes the sensor

Reality:

- AMCL’s `laser_max_range` only limits what AMCL considers; it cannot extend the Gazebo sensor.

Lesson:

- Always check URDF sensor plugin `<range><max>` when localization fails in large spaces.

### 16.2 Mistake: TF authority changes that removed `odom → base_link`

Switching TF publishing between:

- Gazebo diff drive plugin
- EKF

caused at least one regression (robot not moving / not loading).

Lesson:

- In ROS TF, choose a **single authoritative publisher** for `odom → base_link`.

### 16.3 Spec mismatch: `navigation.launch.py` declares args but hardcodes values

In `navigation.launch.py`:

- `map` and `use_sim_time` are declared, but Nav2 bringup is included with:
  - `'map': str(default_map_path)`
  - `'use_sim_time': 'true'`

This means the launch arguments are not actually used.

Lesson:

- Always wire declared `LaunchConfiguration` into included launches.

(If you want, I can fix this in code in a small follow-up commit, but this Phase 4 report documents the current state.)

### 16.4 Non-intuitive result: higher EKF frequency was less stable

You empirically validated:

- 30Hz EKF was less stable than 5Hz in this setup

Lesson:

- More updates ≠ more stability. CPU load + timing jitter can dominate.

---

## 17. Repro instructions (final)

### 17.1 Build

```bash
cd ~/ros2_ws
colcon build --packages-select exercise_launch
source ~/ros2_ws/install/setup.bash
```

### 17.2 Run

Using the wrapper script (recommended on Pop!_OS + NVIDIA):

```bash
~/ros2_ws/src/exercise_launch/scripts/run_navigation.sh
```

Or directly:

```bash
ros2 launch exercise_launch navigation.launch.py
```

### 17.3 Operate

In RViz:

- Fixed Frame is `map` (already set in config)
- Use **“2D Goal Pose”** tool to click a goal
- If needed, use **“2D Pose Estimate”** to reset AMCL pose

---

## 18. Appendix A — Default vs final parameter deltas (high-signal)

This appendix compares the reference template `nav2_params.yaml.backup_default` to the final `nav2_params.yaml`.

### 18.1 Frame IDs

- `amcl.base_frame_id`
  - default: `base_footprint`
  - final: `base_link`

- `collision_monitor.base_frame_id`
  - default: `base_footprint`
  - final: `base_link`

### 18.2 Controller

- default controller plugin:
  - `nav2_mppi_controller::MPPIController`

- final controller plugin:
  - `nav2_regulated_pure_pursuit_controller::RegulatedPurePursuitController`

### 18.3 Costmap radii

- `robot_radius`
  - default: `0.22`
  - final: `0.25` (Phase 4 requirement)

- `inflation_radius`
  - default: `0.70`
  - final: `0.45` (Phase 4 requirement)

- `cost_scaling_factor`
  - default: `3.0`
  - final: `5.0` (Phase 4 requirement)

### 18.4 AMCL sensor/model changes

- default:
  - `laser_max_range: 100.0`, `laser_min_range: -1.0` (template placeholders)

- final:
  - `laser_max_range: 10.0`
  - `laser_min_range: 0.4`

This aligns with the URDF lidar plugin.

---

## 19. Appendix B — Key log excerpts (terminal 17)

### 19.1 Startup TF missing

- `Timed out waiting for transform … Invalid frame ID "odom" … frame does not exist`

### 19.2 AMCL message filter stress

- `Message Filter dropping message … queue is full`
- `… timestamp … earlier than all the data in the transform cache`

### 19.3 Goal success

- `controller_server: Reached the goal!`
- `bt_navigator: Goal succeeded`
- `nav2_goal_bridge: ✓ Navigation goal succeeded!`

### 19.4 Planner failures on far goals

- `GridBased plugin failed to plan … Failed to create plan with tolerance of: 0.500000`

---

## 20. Final Phase 4 status

At the end of Phase 4, the project has:

- A working Nav2 pipeline with AMCL localization on the saved house map.
- RViz-based operator interaction (click-to-goal) via a dedicated bridge.
- Stable-enough localization for multiple goal commands, with known limitations under collision/wheel slip and when setting infeasible goals.

**End of Phase 4 report.**
