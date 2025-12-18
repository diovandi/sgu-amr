#!/bin/bash
# Verify SLAM Toolbox is receiving all required inputs
# Run this while SLAM is running and robot is stationary, then after moving

echo "=== SLAM Input Verification ==="
echo ""

echo "1. Checking if SLAM node is running..."
if ros2 node list | grep -q "slam_toolbox"; then
    echo "   ✓ slam_toolbox node is running"
    echo ""
    echo "   SLAM node subscribers and publishers:"
    ros2 node info /slam_toolbox 2>&1 | grep -A 20 -E "(Subscribers|Publishers)"
else
    echo "   ✗ slam_toolbox node is NOT running"
    exit 1
fi

echo ""
echo "2. Checking TF transform odom -> base_link (required by SLAM):"
if timeout 2 ros2 run tf2_ros tf2_echo odom base_link 2>&1 | grep -q "At time"; then
    echo "   ✓ Transform is available"
    timeout 1 ros2 run tf2_ros tf2_echo odom base_link 2>&1 | grep -E "(Translation|Rotation)" | head -2
else
    echo "   ✗ Transform NOT available - SLAM cannot work without this!"
fi

echo ""
echo "3. Checking /scan topic (required by SLAM):"
if timeout 1 ros2 topic hz /scan 2>&1 | grep -q "average rate"; then
    RATE=$(timeout 1 ros2 topic hz /scan 2>&1 | grep "average rate" | awk '{print $3}')
    echo "   ✓ /scan is publishing at ${RATE} Hz"
    
    echo ""
    echo "   Checking scan data quality..."
    SCAN_MSG=$(timeout 1 ros2 topic echo /scan --once 2>&1)
    if echo "$SCAN_MSG" | grep -q "frame_id: laser_link"; then
        TOTAL_RANGES=$(echo "$SCAN_MSG" | grep -A 500 "ranges:" | grep -E "^[0-9]+\." | wc -l)
        INF_RANGES=$(echo "$SCAN_MSG" | grep -A 500 "ranges:" | grep -c "\.inf")
        VALID_RANGES=$((TOTAL_RANGES - INF_RANGES))
        echo "   Total ranges: $TOTAL_RANGES"
        echo "   Valid ranges (not infinity): $VALID_RANGES"
        echo "   Infinity ranges: $INF_RANGES"
        
        if [ $VALID_RANGES -lt 10 ]; then
            echo "   ⚠ WARNING: Very few valid ranges - SLAM may not be able to process scans"
        else
            echo "   ✓ Scan has sufficient valid data"
        fi
    fi
else
    echo "   ✗ /scan is NOT publishing - SLAM cannot work without this!"
fi

echo ""
echo "4. Checking if robot has moved (SLAM needs movement to start):"
echo "   Current odom->base_link position:"
CURRENT_POS=$(timeout 1 ros2 run tf2_ros tf2_echo odom base_link 2>&1 | grep "Translation:" | head -1)
echo "   $CURRENT_POS"
echo ""
echo "   NOTE: SLAM requires robot to move at least 5cm (minimum_travel_distance)"
echo "   to start processing scans and publishing map."

echo ""
echo "=== Verification Complete ==="
echo ""
echo "If all checks pass but map still not publishing:"
echo "  1. Drive the robot at least 5cm forward"
echo "  2. Wait a few seconds"
echo "  3. Check /map topic again: ros2 topic hz /map"
