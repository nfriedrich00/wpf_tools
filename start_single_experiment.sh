#! /bin/bash

SESSION="Experiment"
max_runtime=300
config_filepath="~/Documents/ros2/wpf_ws/install/wpf_tools/share/wpf_tools/config/waypoint_follower_config.yaml"
results_dir="~/Documents/wpf/logs"
output_file="~/Documents/wpf/results.yaml"
sources=( "/opt/ros/iron/setup.bash" "/home/gjaeger/Documents/Programming/ros2_home_iron/Documents/ros2/dmc_11_ws/install/setup.bash" "/home/gjaeger/Documents/Programming/ros2_home_iron/Documents/ros2/wpf_ws/install/setup.bash" )
init_sources=0

# parse arguments to the script
# optionally accept a new path for the config_filepath, max_runtime, session_name and sources
while getopts "c:t:s:r:" opt; do
    case ${opt} in
        c )
            config_filepath=$OPTARG
            ;;
        t )
            max_runtime=$OPTARG
            ;;
        s )
            # if this is the first source, clear the sources array
            if [ $init_sources -eq 0 ]; then
                sources=()
                init_sources=1
            fi
            
            # append to the sources array
            sources+=("$OPTARG")
            ;;
        r )
            results_dir=$OPTARG
            ;;

        o) 
            output_file=$OPTARG
            ;;
        \? )
            echo "Usage: cmd [-c config_filepath] [-t max_runtime] [-s sources]"
            ;;
    esac
done

# set handler for SIGINT and kill all tmux sessions starting with $SESSION
trap 'echo "SIGINT detected in start_single_experiment.sh! Exiting...";tmux kill-session -t "$SESSION"; sleep 10; exit 1' SIGINT

# define function to run evaluation
function analyze_data {
    echo "Analyzing data..."
    ros2 run wpf_tools analyze_data.py --ros-args -p logs_path:=$results_dir -p results_path:=$output_file
}

# function to check whether all .yaml-files in results_dir are written
function check_results {
    echo "Waiting for results..."

    # loop recusively through all files in results_dir and recurse if a directory is found
    for file in $(find $results_dir -type f -name "*.yaml"); do
        # check if any process is writing to the file
        while lsof $file; do
            echo "File $file is still being written to..."
            sleep 1
        done
    done

    return 0
}

# function to remove results_dir and all its contents
function remove_results {
    echo "Removing results..."
    rm -rf $results_dir
}

# print all arguments for debugging
echo "config_filepath: $config_filepath"
echo "max_runtime: $max_runtime"
for src in "${sources[@]}"
do
  echo "source: $src"
done


# check if path exists and source them
if [ -f "$config_filepath" ]; then
    echo "Config file found at $config_filepath"
else
    echo "Config file not found at $config_filepath"
    exit 1
fi

# source all files in the sources array
for src in "${sources[@]}"
do
    p=$(realpath "$src")
    echo "Sourcing $p"
    if [ -f "$p" ]; then
        source $p
    else
        echo "Source file not found at $p"
        exit 1
    fi
done

# start the experiment if it has not been started yet
started=0
session_name="$SESSION"
i=0
while [ $started -eq 0 ]; do

    # find a unique session name
    while tmux has-session -t "$session_name"; do
        ((i++))
        session_name="$SESSION-$i"
    done

    echo "Unique session name found: $session_name"

    # check size of output.log and delete if it is too big
    if [ -f output.log ]; then
        size=$(du -k output.log | cut -f1)
        if [ $size -gt 100000 ]; then
            echo "output.log is too big. Deleting..."
            rm output.log
        fi
    fi

    # start the experiment
    tmux new-session -d -s "$session_name" -x "$(tput cols)" -y "$(tput lines)" \; pipe-pane -o 'cat >>output.log'
    tmux rename-window -t 0 'window 0'

    sleep 1
    echo "Start experiment" 
    tmux send-keys -t 'window 0' 'ros2 launch wpf_tools waypoint_follower.launch.py run_headless:=True' C-m
    start_time=$(date +%s)

    sleep 20
    # after 20 seconds everything should be running fine
    if [ -z "$(ros2 topic list | grep 'status/gazebo/OK')" ]; then
        echo "Gazebo did not start. Restarting..."
        tmux send-keys -t 'window 0' C-c
        sleep 10
        tmux kill-session -t "$session_name"
        sleep 5 # some nodes keep running for some time
        continue
    fi
    if [ -z "$(ros2 topic list | grep 'status/localization/OK')" ]; then
        echo "Localization is not working correctly. Restarting..."
        tmux send-keys -t 'window 0' C-c
        sleep 10
        tmux kill-session -t "$session_name"
        sleep 5 # some nodes keep running for some time
        continue
    fi
    if [ -z "$(ros2 topic list | grep 'status/navigation/OK')" ]; then
        echo "Navigation is not working correctly. Restarting..."
        tmux send-keys -t 'window 0' C-c
        sleep 10
        tmux kill-session -t "$session_name"
        sleep 5 # some nodes keep running for some time
        continue
    fi

    started=1
done

# wait for the experiment to finish
elapsed_time=$(($(date +%s) - start_time))
while [ $elapsed_time -lt $max_runtime ]; do
    if ! [ -z "$(ros2 topic list | grep 'status/goal_checker/OK')" ]; then
        echo "Goal reached after $elapsed_time seconds"
        echo "Ending..."
        tmux send-keys -t 'window 0' C-c
        sleep 10
        # wait for results to be written
        check_results

        # kill session
        tmux kill-session -t "$session_name"
        sleep 5 # some nodes keep running for some time
        
        # analyze data
        analyze_data
        remove_results
        exit 0
    fi
    elapsed_time=$(($(date +%s) - start_time))
done

# timeout
echo "Timeout, killing..."
tmux send-keys -t 'window 0' C-c
sleep 10
tmux kill-session -t "$session_name"
sleep 5 # some nodes keep running for some time
remove_results
exit 1