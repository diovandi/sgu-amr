#!/bin/bash
# Diagnostic script to check SLAM-related topics and transforms
# Run this while SLAM is running to verify all components are working

echo "=== SLAM Topic Diagnostics ==="
echo ""

echo "1. Checking if topics are publishing..."
echo "   /scan:"
timeout 2 ros2 topic hz /scan 2>&1 | head -3 || echo "   ❌ /scan not publishing"
echo "   /odom:"
timeout 2 ros2 topic hz /odom 2>&1 | head -3 || echo "   ❌ /odom not publishing"
echo "   /imu:"
timeout 2 ros2 topic hz /imu 2>&1 | head -3 || echo "   ❌ /imu not publishing"
echo "   /odometry/filtered:"
timeout 2 ros2 topic hz /odometry/filtered 2>&1 | head -3 || echo "   ❌ /odometry/filtered not publishing"
echo "   /map:"
timeout 2 ros2 topic hz /map 2>&1 | head -3 || echo "   ❌ /map not publishing (normal if SLAM just started)"
echo ""

echo "2. Checking TF transforms..."
echo "   odom -> base_link:"
timeout 2 ros2 run tf2_ros tf2_echo odom base_link 2>&1 | head -5 || echo "   ❌ Transform not available"
echo "   map -> odom:"
timeout 2 ros2 run tf2_ros tf2_echo map odom 2>&1 | head -5 || echo "   ❌ Transform not available (normal if SLAM just started)"
echo ""

echo "3. Checking IMU data sample..."
timeout 1 ros2 topic echo /imu --once 2>&1 | head -10 || echo "   ❌ Could not read IMU data"
echo ""

echo "4. Checking EKF filtered odometry sample..."
timeout 1 ros2 topic echo /odometry/filtered --once 2>&1 | head -15 || echo "   ❌ Could not read filtered odometry"
echo ""

echo "5. Checking raw odometry sample (from Gazebo)..."
timeout 1 ros2 topic echo /odom --once 2>&1 | head -15 || echo "   ❌ Could not read raw odometry"
echo ""

echo "6. Checking LIDAR scan sample..."
timeout 1 ros2 topic echo /scan --once 2>&1 | head -20 || echo "   ❌ Could not read LIDAR scan"
echo ""

echo "7. Checking clock topic..."
timeout 1 ros2 topic echo /clock --once 2>&1 | head -5 || echo "   ❌ Could not read clock"
echo ""

echo "=== Diagnostics Complete ==="
