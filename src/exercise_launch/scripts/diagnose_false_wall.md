# Potential Causes of False Wall Detection (Beyond Self-Detection)

## 1. **Ground Plane Detection**
- **Issue**: LIDAR at z=0.5m might be detecting the ground plane (z=0) as a wall
- **Symptom**: Perpendicular wall appears at fixed distance in front of robot
- **Check**: Inspect scan ranges - if they're all ~0.5m, it's detecting the ground
- **Fix**: Increase LIDAR height further or add ground plane filtering

## 2. **GPU LIDAR Rendering Artifacts**
- **Issue**: `gpu_lidar` in Gazebo Harmonic can have rendering artifacts
- **Symptom**: False detections from rendering engine issues
- **Check**: Verify OpenGL version: `glxinfo | grep "OpenGL version"`
- **Fix**: Update GPU drivers or switch to CPU lidar (`lidar` instead of `gpu_lidar`)

## 3. **Frame ID Mismatch**
- **Issue**: Scan message might have wrong `header.frame_id`
- **Symptom**: RViz transforms scan points incorrectly, creating false walls
- **Check**: Run `./inspect_scan.sh` to verify frame_id
- **Fix**: Ensure `gz_frame_id: laser_link` matches scan message `header.frame_id`

## 4. **TF Transform Timing Issues**
- **Issue**: RViz might be using stale or incorrect transforms
- **Symptom**: Scan points appear in wrong location
- **Check**: Verify TF tree: `ros2 run tf2_tools view_frames`
- **Fix**: Ensure EKF is publishing `odom->base_link` and `robot_state_publisher` is publishing `base_link->laser_link`

## 5. **World Geometry Issues**
- **Issue**: Invisible collision geometry in world file
- **Symptom**: Wall appears even when robot is in open space
- **Check**: Inspect world file for unexpected geometry
- **Fix**: Remove or adjust world geometry

## 6. **Scan Message Header Timestamp**
- **Issue**: Scan timestamps might be incorrect, causing transform lookup failures
- **Symptom**: "timestamp on the message is earlier than all the data in the transform cache"
- **Check**: Verify scan message timestamps match clock topic
- **Fix**: Ensure `use_sim_time: true` is set everywhere

## 7. **LIDAR Sensor Pose in Gazebo**
- **Issue**: Sensor pose might be incorrect relative to joint
- **Symptom**: Scan origin is wrong, creating offset walls
- **Check**: Verify sensor `<pose>` matches joint origin
- **Fix**: Adjust sensor pose or joint origin

## 8. **Bridge Frame ID Preservation**
- **Issue**: `ros_gz_bridge` might not preserving frame_id correctly
- **Symptom**: Scan has wrong frame_id after bridging
- **Check**: Compare Gazebo topic frame_id vs ROS topic frame_id
- **Fix**: Verify bridge configuration or add frame_id remapping
