#! /bin/bash

tmux new -s ros_session -d
tmux split-window -h -t ros_session
tmux split-window -v -t ros_session:0.1
tmux split-window -v -t ros_session:0.0

tmux send-keys -t ros_session:0.0 '. ./setup.sh' C-m
tmux send-keys -t ros_session:0.0 'roscore' C-m
tmux send-keys -t ros_session:0.1 'sleep 3' C-m #to ensure roscore runs first
tmux send-keys -t ros_session:0.1 'roslaunch turtlebot_bringup minimal.launch' C-m
tmux send-keys -t ros_session:0.2 'sleep 5' C-m #to ensure roscore runs first
tmux send-keys -t ros_session:0.2 'roslaunch turtlebot_dashboard turtlebot_dashboard.launch' C-m

tmux attach -t ros_session
