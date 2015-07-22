#!/bin/bash

SESSION=turtlebot

DATABASE=/var/local/mongodb_store
MAP=/home/turtlebot/maps/lg_20140618/cropped.yaml
MOVE_BASE_CUSTOM_PARAMS=/home/turtlebot/nav_params.yaml
TOPOLOGICAL_MAP=lg_coordination

message () { tmux send-keys " #   ===  $1  ===" C-m;  };
split_4 () { tmux split-window -v; tmux select-pane -t 1; tmux split-window -h; tmux select-pane -t 0; tmux split-window -h; tmux select-pane -t 0; };


tmux -2 new-session -d -s $SESSION
# Setup a window for tailing log files
tmux new-window -t $SESSION:0 -n 'roscore'
tmux new-window -t $SESSION:1 -n 'mongo'
tmux new-window -t $SESSION:2 -n 'robot'
tmux new-window -t $SESSION:3 -n 'move_base'
tmux new-window -t $SESSION:4 -n 'pose_publisher'
tmux new-window -t $SESSION:5 -n 'topo_nav'
tmux new-window -t $SESSION:6 -n 'discovery'
tmux new-window -t $SESSION:7 -n 'sync'
tmux new-window -t $SESSION:8 -n 'planner'
tmux new-window -t $SESSION:9 -n 'random_goal'

tmux select-window -t $SESSION:0
tmux split-window -v
tmux select-pane -t 0
tmux send-keys "roscore" C-m
tmux resize-pane -U 50
tmux select-pane -t 1
tmux send-keys "htop" C-m

tmux select-window -t $SESSION:1
tmux send-keys " clear" C-m
message "MongoDB message_store"
tmux send-keys "roslaunch mongodb_store mongodb_store.launch db_path:=$DATABASE"

tmux select-window -t $SESSION:2
tmux send-keys " clear" C-m
message "Robot drivers"
tmux send-keys "roslaunch turtlebot_bringup minimal.launch"

tmux select-window -t $SESSION:3
tmux send-keys " clear" C-m
message "move_base"
tmux send-keys "roslaunch turtlebot_navigation amcl_demo.launch custom_param_file:=$MOVE_BASE_CUSTOM_PARAMS map_file:=$MAP"

tmux select-window -t $SESSION:4
tmux send-keys " clear" C-m
message "pose_publisher"
tmux send-keys "rosrun robot_pose_publisher robot_pose_publisher"

tmux select-window -t $SESSION:5
tmux send-keys " clear" C-m
message "Topological navigation"
tmux send-keys "roslaunch topological_navigation topological_navigation.launch map:=$TOPOLOGICAL_MAP"

tmux select-window -t $SESSION:6
tmux send-keys " clear" C-m
message "Master discovery"
tmux send-keys "rosrun master_discovery_fkie master_discovery"

tmux select-window -t $SESSION:7
tmux send-keys " clear" C-m
message "Master sync"
tmux send-keys "rosrun master_sync_fkie master_sync _sync_nodes:=['/path_planner'] _sync_topics:=['/robot1/position_info']"

tmux select-window -t $SESSION:8
tmux send-keys " clear" C-m
message "Path planner"
tmux send-keys "roslaunch dijkstra path_planner.launch"

tmux select-window -t $SESSION:9
tmux send-keys " clear" C-m
message "Random Goal"
tmux send-keys "rosrun dijkstra random_goal.py"


# Set default window
tmux select-window -t $SESSION:0

# Attach to session
tmux -2 attach-session -t $SESSION



