#! /bin/bash

max_runtime=300
config_filepath=~/Documents/ros2/wpf_ws/install/wpf_tools/share/wpf_tools/config/waypoint_follower_config.yaml

# get the path of the script
SCRIPTPATH="$( cd "$(dirname "$0")" >/dev/null 2>&1 ; pwd -P )"

# store current path
current_path=$(pwd)

# set handler for SIGINT and kill all tmux sessions starting with $SESSION
trap 'echo "SIGINT detected in start_experiments.sh! Exiting..."; cd "$current_path"; sleep 10; exit 1' SIGINT

# switch to the script path
cd $SCRIPTPATH

# execute experiments
while true; do
    # call start_single_experiment.sh with the same arguments
    ./start_single_experiment.sh -c $config_filepath -t $max_runtime
done

# switch back to the original path
cd $current_path
