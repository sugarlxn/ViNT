#!/bin/bash

#./create_topomap.sh <topomap_name> <bag_filename>

# Create a new tmux session
session_name="gnm_locobot_$(date +%s)"
tmux new-session -d -s $session_name

# Split the window into three panes
tmux selectp -t 0    # select the first (0) pane
tmux splitw -v -p 50 # split it into two halves

# Run the create_topoplan.py script with command line args in the second pane
tmux select-pane -t 0
tmux send-keys "conda activate nomad_train" Enter
tmux send-keys "source /opt/ros/foxy/setup.bash" Enter
tmux send-keys "python my_create_topomap.py --dt 1 --dir $1" Enter

# Change the directory to ../topomaps/bags and run the rosbag play command in the third pane
tmux select-pane -t 1
tmux send-keys "cd ../bag" Enter
tmux send-keys "ros2 bag play -r 1.5 $2" # feel free to change the playback rate to change the edge length in the graph

# Attach to the tmux session
tmux -2 attach-session -t $session_name