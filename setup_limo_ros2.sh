#!/bin/bash
# Limo ROS2 Humble setup script

set -e  # exit immediately on error

echo "=== Limo ROS2 Humble Setup ==="

# Check for ROS2 Humble
if [ ! -d "/opt/ros/humble" ]; then
    echo "ROS2 Humble not found. Please install ROS2 Humble manually first."
    echo "Installation guide: https://docs.ros.org/en/humble/Installation/Alternatives/Ubuntu-Development-Setup.html"
    exit 1
fi

# Update system
echo "Updating system..."
sudo apt update  -y

# Install dependencies
echo "Installing required ROS2 packages..."
sudo apt install -y \
  ros-humble-joint-state-publisher-gui \
  ros-humble-rqt-robot-steering \
  ros-humble-teleop-twist-keyboard \
  ros-humble-navigation2 \
  ros-humble-nav2-bringup \ 
  ros-humble-topic-tools

# Create workspace
echo "Creating workspace..."
# mkdir -p ~/limo_ws/src
# cd ~/limo_ws/src

# Clone repositories
if [ ! -d "limo_ros2" ]; then
    git clone https://github.com/idra-lab/limo_ros2.git
fi
if [ ! -d "gz_ros2_control" ]; then
    git clone https://github.com/tamasso-parec/gz_ros2_control.git
fi

cd ~/limo_ws

# Source ROS2 setup
source /opt/ros/humble/setup.bash

# Install rosdep dependencies
echo "Installing rosdep dependencies..."
sudo apt install -y python3-rosdep
sudo rosdep init 2>/dev/null || true
rosdep update
rosdep install --from-paths src --ignore-src -r -y

# Build workspace
echo "Building workspace..."
colcon build

# Source workspace
echo "Sourcing workspace..."
source install/setup.bash

echo "=== Installation Complete ==="
echo
echo "To verify installation, run:"
echo "  ros2 launch limo_car ackermann_gazebo.launch.py"
echo
echo "In another terminal:"
echo "  source /opt/ros/humble/setup.bash"
echo "  source ~/limo_ws/install/setup.bash"
echo "  ros2 run rqt_robot_steering rqt_robot_steering"
