#!/bin/bash

SESSION=simulator

DATABASE=/var/local/mongodb_store
MAP=/home/pedrolge/Documents/Summer/lg_20140618/cropped.yaml
MOVE_BASE_CUSTOM_PARAMS=/home/turtlebot/nav_params.yaml
TOPOLOGICAL_MAP=lg_june14

message () { tmux send-keys " #   ===  $1  ===" C-m;  };;


# Setup a window for tailing log files
tmux new-window -t $SESSION:0 -n 'roscore'
tmux new-window -t $SESSION:1 -n 'mongo'
tmux new-window -t $SESSION:2 -n 'morse'
tmux new-window -t $SESSION:3 -n 'gui'
tmux new-window -t $SESSION:4 -n 'bringup'
tmux new-window -t $SESSION:5 -n 'rviz'
tmux new-window -t $SESSION:6 -n 'path_planner'
tmux new-window -t $SESSION:6 -n 'random_goal'

tmux select-window -t $SESSION:0
tmux send-keys " clear" C-m
message "Roscore"
tmux send-keys "roscore" C-m

tmux select-window -t $SESSION:1
tmux send-keys " clear" C-m
message "MongoDB message_store"
tmux send-keys "roslaunch mongodb_store mongodb_store.launch"

tmux select-window -t $SESSION:2
tmux send-keys " clear" C-m
message "Strands Morse"
tmux send-keys "roslaunch strands_morse bham_cs_morse.launch"

tmux select-window -t $SESSION:3
tmux send-keys " clear" C-m
message "Strands UI"
tmux send-keys "roslaunch strands_ui strands_ui.launch"

tmux select-window -t $SESSION:4
tmux send-keys " clear" C-m
message "pose_publisher"
tmux send-keys "roslaunch strands_bringup strands_navigation.launch map:=$MAP no_go_map:=$MAP topological_map:=$TOPOLOGICAL_MAP"

tmux select-window -t $SESSION:5
tmux send-keys " clear" C-m
message "RVIZ"
tmux send-keys "rosrun rviz rviz"

tmux select-window -t $SESSION:6
tmux send-keys " clear" C-m
message "Python Path Planner"
tmux send-keys "rosrun dijkstra path_planner.py"

tmux select-window -t $SESSION:7
tmux send-keys " clear" C-m
message "Python Random Goal"
tmux send-keys "rosrun dijkstra random_goal.py"

# Set default window
tmux select-window -t $SESSION:1

# Attach to session
tmux -2 attach-session -t $SESSION

