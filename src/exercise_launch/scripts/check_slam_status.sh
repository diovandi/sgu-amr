#!/bin/bash
# Comprehensive SLAM status check
# Verifies all prerequisites for SLAM to publish map

echo "=== SLAM Status Check ==="
echo ""

echo "1. Required Topics Status:"
echo -n "   /scan: "
if timeout 1 ros2 topic hz /scan 2>&1 | grep -q "average rate"; then
    RATE=$(timeout 1 ros2 topic hz /scan 2>&1 | grep "average rate" | awk '{print $3}')
    echo "✓ Publishing at ${RATE} Hz"
else
    echo "✗ NOT publishing"
fi

echo -n "   /odom: "
if timeout 1 ros2 topic hz /odom 2>&1 | grep -q "average rate"; then
    RATE=$(timeout 1 ros2 topic hz /odom 2>&1 | grep "average rate" | awk '{print $3}')
    echo "✓ Publishing at ${RATE} Hz"
else
    echo "✗ NOT publishing"
fi

echo -n "   /imu: "
if timeout 1 ros2 topic hz /imu 2>&1 | grep -q "average rate"; then
    RATE=$(timeout 1 ros2 topic hz /imu 2>&1 | grep "average rate" | awk '{print $3}')
    echo "✓ Publishing at ${RATE} Hz"
else
    echo "✗ NOT publishing"
fi

echo -n "   /odometry/filtered: "
if timeout 1 ros2 topic hz /odometry/filtered 2>&1 | grep -q "average rate"; then
    RATE=$(timeout 1 ros2 topic hz /odometry/filtered 2>&1 | grep "average rate" | awk '{print $3}')
    echo "✓ Publishing at ${RATE} Hz"
else
    echo "✗ NOT publishing (EKF may not be running)"
fi

echo -n "   /map: "
if timeout 1 ros2 topic hz /map 2>&1 | grep -q "average rate"; then
    RATE=$(timeout 1 ros2 topic hz /map 2>&1 | grep "average rate" | awk '{print $3}')
    echo "✓ Publishing at ${RATE} Hz"
else
    echo "✗ NOT publishing (SLAM may not be running or robot hasn't moved)"
fi

echo ""
echo "2. TF Transform Status:"
echo -n "   odom -> base_link: "
if timeout 1 ros2 run tf2_ros tf2_echo odom base_link 2>&1 | grep -q "At time"; then
    echo "✓ Available"
else
    echo "✗ NOT available (EKF may not be publishing)"
fi

echo -n "   base_link -> laser_link: "
if timeout 1 ros2 run tf2_ros tf2_echo base_link laser_link 2>&1 | grep -q "At time"; then
    echo "✓ Available"
else
    echo "✗ NOT available (robot_state_publisher issue)"
fi

echo -n "   map -> odom: "
if timeout 1 ros2 run tf2_ros tf2_echo map odom 2>&1 | grep -q "At time"; then
    echo "✓ Available (SLAM is running)"
else
    echo "✗ NOT available (SLAM may not have started or robot hasn't moved)"
fi

echo ""
echo "3. SLAM Node Status:"
if ros2 node list | grep -q "slam_toolbox"; then
    echo "   ✓ slam_toolbox node is running"
    echo "   Node info:"
    ros2 node info /slam_toolbox 2>&1 | grep -E "(Subscribers|Publishers)" | head -4
else
    echo "   ✗ slam_toolbox node is NOT running"
fi

echo ""
echo "4. EKF Node Status:"
if ros2 node list | grep -q "ekf_filter_node"; then
    echo "   ✓ ekf_filter_node is running"
else
    echo "   ✗ ekf_filter_node is NOT running"
fi

echo ""
echo "5. Recent Scan Data Check:"
SCAN_MSG=$(timeout 1 ros2 topic echo /scan --once 2>&1)
if echo "$SCAN_MSG" | grep -q "frame_id: laser_link"; then
    RANGE_COUNT=$(echo "$SCAN_MSG" | grep -A 100 "ranges:" | grep -E "^[0-9]+\." | wc -l)
    INF_COUNT=$(echo "$SCAN_MSG" | grep -A 100 "ranges:" | grep -c "\.inf")
    VALID_RANGES=$((RANGE_COUNT - INF_COUNT))
    echo "   Scan has $VALID_RANGES valid ranges out of $RANGE_COUNT total"
    if [ $VALID_RANGES -lt 10 ]; then
        echo "   ⚠ WARNING: Very few valid ranges - LIDAR may not be detecting environment"
    fi
else
    echo "   ✗ Could not read scan message"
fi

echo ""
echo "=== Status Check Complete ==="
echo ""
echo "NOTE: If /map is not publishing, ensure:"
echo "  1. Robot has moved at least 5cm (minimum_travel_distance)"
echo "  2. EKF is publishing /odometry/filtered"
echo "  3. /scan topic has valid data"
echo "  4. SLAM node is running (check with: ros2 node list)"
