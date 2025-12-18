#!/bin/bash
# Diagnostic script to inspect LIDAR scan messages
# Checks frame_id, range values, and transform availability

echo "=== LIDAR Scan Message Inspection ==="
echo ""

echo "1. Checking scan topic info..."
ros2 topic info /scan --verbose 2>&1 | head -20
echo ""

echo "2. Checking scan message header (frame_id)..."
timeout 2 ros2 topic echo /scan --once 2>&1 | grep -A 5 "header:" || echo "   Could not read scan message"
echo ""

echo "3. Checking scan ranges (first 20 values)..."
timeout 2 ros2 topic echo /scan --once 2>&1 | grep -A 25 "ranges:" | head -25 || echo "   Could not read scan ranges"
echo ""

echo "4. Checking if laser_link frame exists in TF tree..."
timeout 2 ros2 run tf2_ros tf2_echo base_link laser_link 2>&1 | head -10 || echo "   Transform not available"
echo ""

echo "5. Checking scan message rate..."
timeout 3 ros2 topic hz /scan 2>&1 | head -5 || echo "   Topic not publishing"
echo ""

echo "6. Checking for invalid/infinity ranges..."
timeout 2 ros2 topic echo /scan --once 2>&1 | grep -E "(inf|nan|-inf)" | head -5 || echo "   No invalid ranges found"
echo ""

echo "=== Inspection Complete ==="
