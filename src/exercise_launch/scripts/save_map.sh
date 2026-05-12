#!/bin/bash
# Script to save the SLAM-generated map to house_map.pgm and house_map.yaml
# Usage: ./save_map.sh [output_directory]

# Usage: ./save_map.sh [output_directory]
# If no directory specified, saves to data/final-project/maps/ in this workspace.
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_ROOT="$(cd "${SCRIPT_DIR}/../../.." && pwd)"
export SGU_AMR_WORKSPACE="${SGU_AMR_WORKSPACE:-$WORKSPACE_ROOT}"

if [ -z "$1" ]; then
    OUTPUT_DIR="$SGU_AMR_WORKSPACE/data/final-project/maps"
else
    OUTPUT_DIR="$1"
fi
MAP_NAME="house_map"

# Ensure output directory exists
mkdir -p "$OUTPUT_DIR"

# Source ROS 2 workspace
source /opt/ros/jazzy/setup.bash
source "$SGU_AMR_WORKSPACE/install/setup.bash"

echo "=== Saving SLAM Map ==="
echo ""

# Check if /map topic exists
echo "Checking for /map topic..."
if ! ros2 topic list 2>/dev/null | grep -q "^/map$"; then
    echo "✗ ERROR: /map topic not found in topic list"
    echo "   Make sure SLAM is running: ./src/exercise_launch/scripts/run_slam.sh"
    exit 1
fi

echo "✓ /map topic found"
echo "   Note: map_saver_cli will wait for the next map message..."

# Get the full path for the map file
MAP_PATH="${OUTPUT_DIR}/${MAP_NAME}"

echo ""
echo "Saving map to: ${MAP_PATH}.pgm and ${MAP_PATH}.yaml"
echo ""

# Save the map using map_saver_cli
# -f specifies the base filename (without extension)
# Note: map_saver_cli will wait for one map message, so we don't need to wait longer
echo "Waiting for map message and saving..."
ros2 run nav2_map_server map_saver_cli -f "$MAP_PATH" --ros-args -p use_sim_time:=true

if [ $? -eq 0 ]; then
    echo ""
    echo "✓ Map saved successfully!"
    echo ""
    echo "Files created:"
    echo "  - ${MAP_PATH}.pgm"
    echo "  - ${MAP_PATH}.yaml"
    echo ""
    
    # Verify files exist
    if [ -f "${MAP_PATH}.pgm" ] && [ -f "${MAP_PATH}.yaml" ]; then
        echo "✓ Both files verified"
        ls -lh "${MAP_PATH}".{pgm,yaml} 2>/dev/null
    else
        echo "⚠ WARNING: Some files may be missing"
    fi
else
    echo ""
    echo "✗ ERROR: Failed to save map"
    exit 1
fi
