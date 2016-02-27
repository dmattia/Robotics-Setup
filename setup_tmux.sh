#! /bin/bash

tmux new -s ros_session -d
tmux split-window -h -t ros_session
tmux split-window -v -t ros_session:0.1
tmux split-window -v -t ros_session:0.2
tmux split-window -v -t ros_session:0.3
tmux split-window -v -t ros_session:0.1
tmux split-window -v -t ros_session:0.2

tmux send-keys -t ros_session:0.1 '. ./setup.sh' C-m
tmux send-keys -t ros_session:0.1 'roscore' C-m
tmux send-keys -t ros_session:0.2 'sleep 3' C-m #to ensure roscore runs first
tmux send-keys -t ros_session:0.2 'roslaunch turtlebot_bringup minimal.launch' C-m
tmux send-keys -t ros_session:0.3 'sleep 5' C-m #to ensure roscore runs first
tmux send-keys -t ros_session:0.3 'roslaunch openni_launch openni.launch' C-m
tmux send-keys -t ros_session:0.4 'sleep 7' C-m #to ensure roscore runs first
tmux send-keys -t ros_session:0.4 'rosrun image_view image_view image:=/camera/rgb/image_color' C-m
tmux send-keys -t ros_session:0.5 'sleep 9' C-m #to ensure roscore runs first
tmux send-keys -t ros_session:0.5 'roscd cmvision' C-m
tmux send-keys -t ros_session:0.5 'roslaunch cmvision.launch image:=/camera/rgb/image_color' C-m
tmux send-keys -t ros_session:0.6 'sleep 11' C-m #to ensure roscore runs first
tmux send-keys -t ros_session:0.6 'rosrun cmvision colorgui image:=/camera/rgb/image_color' C-m

tmux attach -t ros_session
