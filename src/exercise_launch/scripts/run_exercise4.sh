#!/bin/bash
# Launch script for Exercise 4 playback on Pop!_OS Cosmic with NVIDIA GPU

export QT_QPA_PLATFORM=xcb
export GDK_BACKEND=x11
export __GLX_VENDOR_LIBRARY_NAME=nvidia
export __NV_PRIME_RENDER_OFFLOAD=1
export __VK_LAYER_NV_optimus=NVIDIA_only
export OGRE_RTT_PREFERRED_MODE=FBO
export MESA_GL_VERSION_OVERRIDE=4.5

xhost +local: 2>/dev/null || true

source /opt/ros/jazzy/setup.bash
source ~/ros2_ws/install/setup.bash

ros2 launch exercise_launch exercise4.launch.py "$@"
























