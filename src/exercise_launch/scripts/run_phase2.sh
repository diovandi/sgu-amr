#!/bin/bash
# Launch script for Phase 2: Sensor Processing
# Custom rudimentary_bot simulation with LIDAR reader node
# Pop!_OS with NVIDIA GPU compatibility

# Ensure we're using X11/XWayland properly
export QT_QPA_PLATFORM=xcb
export GDK_BACKEND=x11

# NVIDIA-specific settings for Wayland compatibility
export __GLX_VENDOR_LIBRARY_NAME=nvidia
export __NV_PRIME_RENDER_OFFLOAD=1
export __VK_LAYER_NV_optimus=NVIDIA_only

# Ogre/Gazebo rendering settings
export OGRE_RTT_PREFERRED_MODE=FBO
export MESA_GL_VERSION_OVERRIDE=4.5

# Disable VSync and buffer issues
export __GL_SYNC_TO_VBLANK=0
export vblank_mode=0

# Compositing/Triple Buffering
export __GL_YIELD="USLEEP"
export __GL_MaxFramesAllowed=1

# Qt OpenGL Backend - force desktop OpenGL
export QT_OPENGL=desktop
export QSG_RENDER_LOOP=basic

# XWayland compatibility - disable DRI3
export LIBGL_DRI3_DISABLE=1

# Allow local X11 connections (needed for some XWayland setups)
xhost +local: 2>/dev/null || true

# Source ROS 2 workspace
source /opt/ros/jazzy/setup.bash
source ~/ros2_ws/install/setup.bash

# Launch Phase 2 (simulation + LIDAR reader)
ros2 launch exercise_launch phase2.launch.py "$@"
