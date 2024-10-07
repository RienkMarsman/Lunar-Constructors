#!/bin/bash

# File: launch_rover_and_control.sh
# Description: Launches Gazebo with a rover and starts the teleop control panel

# Check if tmux is installed
if ! command -v tmux &> /dev/null; then
    echo "tmux is not installed. Please install it first."
    exit 1
fi

# Start a new tmux session
tmux new-session -d -s rover_session

# First pane: Launch Gazebo with rover
tmux send-keys -t rover_session:0.0 "source /opt/ros/iron/setup.bash" C-m
tmux send-keys -t rover_session:0.0 "source ~/ws_mobile/install/setup.bash" C-m
tmux send-keys -t rover_session:0.0 "colcon build" C-m
tmux send-keys -t rover_session:0.0 "ros2 launch mobile_robot gazebo_model.launch.py" C-m

# Create a new pane for teleop control
tmux split-window -h -t rover_session:0

# Second pane: Run teleop control
tmux send-keys -t rover_session:0.1 "source /opt/ros/iron/setup.bash" C-m
tmux send-keys -t rover_session:0.1 "sleep 5" C-m  # Wait for Gazebo to start
tmux send-keys -t rover_session:0.1 "ros2 run teleop_twist_keyboard teleop_twist_keyboard" C-m

# Attach to the tmux session
tmux attach-session -t rover_session
