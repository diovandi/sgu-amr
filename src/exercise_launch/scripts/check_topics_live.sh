#!/bin/bash
# Real-time topic monitoring script
# Run this while SLAM is running to see live topic status

echo "=== Live Topic Status (updates every 2 seconds) ==="
echo "Press Ctrl+C to stop"
echo ""

while true; do
    clear
    echo "=== Live Topic Status ==="
    echo "Time: $(date +%H:%M:%S)"
    echo ""
    
    echo "1. Topic Publishing Status:"
    echo -n "   /scan: "
    timeout 1 ros2 topic hz /scan 2>&1 | grep -q "average rate" && echo "✓ Publishing" || echo "✗ NOT publishing"
    
    echo -n "   /odom: "
    timeout 1 ros2 topic hz /odom 2>&1 | grep -q "average rate" && echo "✓ Publishing" || echo "✗ NOT publishing"
    
    echo -n "   /imu: "
    timeout 1 ros2 topic hz /imu 2>&1 | grep -q "average rate" && echo "✓ Publishing" || echo "✗ NOT publishing"
    
    echo -n "   /odometry/filtered: "
    timeout 1 ros2 topic hz /odometry/filtered 2>&1 | grep -q "average rate" && echo "✓ Publishing" || echo "✗ NOT publishing"
    
    echo -n "   /map: "
    timeout 1 ros2 topic hz /map 2>&1 | grep -q "average rate" && echo "✓ Publishing" || echo "✗ NOT publishing"
    
    echo ""
    echo "2. Recent LIDAR Scan Sample (first 5 ranges):"
    timeout 0.5 ros2 topic echo /scan --once 2>&1 | grep -A 5 "ranges:" | head -6 || echo "   Could not read scan"
    
    echo ""
    echo "3. TF Frame Status:"
    echo -n "   odom frame exists: "
    timeout 0.5 ros2 run tf2_ros tf2_echo odom base_link 2>&1 | grep -q "At time" && echo "✓" || echo "✗"
    
    echo -n "   map frame exists: "
    timeout 0.5 ros2 run tf2_ros tf2_echo map odom 2>&1 | grep -q "At time" && echo "✓" || echo "✗"
    
    sleep 2
done
