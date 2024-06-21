#! /bin/bash

SESSION="Experiment"
max_runtime=300
waypoints_filepath="~/Documents/ros2/wpf_ws/src/wpf_tools/config/waypoints_line.yaml"
gnss_error_filepath="~/Documents/ros2/wpf_ws/src/wpf_tools/config/gps_error_simulator_config.yaml"
nav_planner_filepath="~/Documents/ros2/wpf_ws/src/wpf_tools/config/planner_straight_line.yaml"
nav_controller_filepath="~/Documents/ros2/wpf_ws/src/wpf_tools/config/controller_rpp.yaml"
results_dir="~/Documents/wpf/logs"
output_file="~/Documents/wpf/results.yaml"
sources=( "/opt/ros/iron/setup.bash" "/home/gjaeger/Documents/Programming/ros2_home_iron/Documents/ros2/dmc_11_ws/install/setup.bash" "/home/gjaeger/Documents/Programming/ros2_home_iron/Documents/ros2/wpf_ws/install/setup.bash" )
init_sources=0
run_headless=1
quiet=0

# parse arguments to the script
# optionally accept a new path for the config_filepath, max_runtime, session_name and sources
while getopts "w:g:p:c:t:s:r:o:v:q:" opt; do
    case ${opt} in
        w )
            waypoints_filepath=$OPTARG
            ;;
        g )
            gnss_error_filepath=$OPTARG
            ;;
        p )
            nav_planner_filepath=$OPTARG
            ;;
        c )
            nav_controller_filepath=$OPTARG
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
        v)
            # accept 0,1,true,false,True,False,TRUE,FALSE and convert to 0,1
            if [ $OPTARG == "0" ] || [ $OPTARG == "1" ] || [ $OPTARG == "true" ] || [ $OPTARG == "false" ] || [ $OPTARG == "True" ] || [ $OPTARG == "False" ] || [ $OPTARG == "TRUE" ] || [ $OPTARG == "FALSE" ]; then
                if [ $OPTARG == "0" ] || [ $OPTARG == "false" ] || [ $OPTARG == "False" ] || [ $OPTARG == "FALSE" ]; then
                    run_headless=0
                else
                    run_headless=1
                fi
            else
                echo "Invalid value for run_headless: $OPTARG"
                exit 1
            fi
            ;;
        q)
            # accept 0,1,true,false,True,False,TRUE,FALSE and convert to 0,1
            if [ $OPTARG == "0" ] || [ $OPTARG == "1" ] || [ $OPTARG == "true" ] || [ $OPTARG == "false" ] || [ $OPTARG == "True" ] || [ $OPTARG == "False" ] || [ $OPTARG == "TRUE" ] || [ $OPTARG == "FALSE" ]; then
                if [ $OPTARG == "0" ] || [ $OPTARG == "false" ] || [ $OPTARG == "False" ] || [ $OPTARG == "FALSE" ]; then
                    quiet=0
                else
                    quiet=1
                fi
            else
                echo "Invalid value for run_headless: $OPTARG"
                exit 1
            fi
            ;;
        \?)
            echo "Usage: cmd [-w waypoints_filepath] [-g gnss_error_filepath] [-p nav_planner_filepath] [-c nav_controller_filepath] [-t max_runtime] [-s source] [-r results_dir] [-o output_file]"
            exit 1
            ;;
    esac
done

# set handler for SIGINT and kill all tmux sessions starting with $SESSION
trap 'echo "SIGINT detected in start_single_experiment.sh! Exiting...";tmux kill-session -t "$SESSION"; sleep 10; exit 1' SIGINT

# define function to run evaluation
function analyze_data {
    if [ $quiet -eq 0 ]; then
        echo "Analyzing data..."
    fi
    ros2 run wpf_tools analyze_data --ros-args -p logs_path:=$results_dir -p results_path:=$output_file
}

# function to check whether all .yaml-files in results_dir are written
function check_results {
    if [ $quiet -eq 0 ]; then
        echo "Waiting for results..."
    fi

    # loop recusively through all files in results_dir and recurse if a directory is found
    for file in $(find $results_dir -type f -name "*.yaml"); do
        # check if any process is writing to the file
        while lsof $file; do
            if [ $quiet -eq 0 ]; then
                echo "File $file is still being written to..."
            fi
            sleep 1
        done
    done

    return 0
}

# function to remove results_dir and all its contents
function remove_results {
    if [ $quiet -eq 0 ]; then
        echo "Removing results..."
    fi
    rm -rf $results_dir
}

# function for checking if file exists
function check_file {
    if [ -f "$1" ]; then
        if [ $quiet -eq 0 ]; then
            echo "File $1 found"
        fi
        return 0
    else
        echo "File $1 not found"
        return 1
    fi
}

# function for replace ~ in string with $HOME
function resolve_path {
    echo $(eval echo $1)
}

# resolve all file paths
waypoints_filepath=$(resolve_path $waypoints_filepath)
gnss_error_filepath=$(resolve_path $gnss_error_filepath)
nav_planner_filepath=$(resolve_path $nav_planner_filepath)
nav_controller_filepath=$(resolve_path $nav_controller_filepath)
results_dir=$(resolve_path $results_dir)
output_file=$(resolve_path $output_file)

# print all arguments for debugging
if [ $quiet -eq 0 ]; then
    echo "waypoints_filepath: $waypoints_filepath"
    echo "gnss_error_filepath: $gnss_error_filepath"
    echo "nav_planner_filepath: $nav_planner_filepath"
    echo "nav_controller_filepath: $nav_controller_filepath"
    echo "results_dir: $results_dir"
    echo "output_file: $output_file"
    echo "max_runtime: $max_runtime"
    echo "run_headless: $run_headless"
    echo "quiet: $quiet"
    for src in "${sources[@]}"
    do
    echo "source: $src"
    done
fi


# check whether all config files exist
check_file $waypoints_filepath
if [ $? -ne 0 ]; then
    exit 1
fi

check_file $gnss_error_filepath
if [ $? -ne 0 ]; then
    exit 1
fi

check_file $nav_planner_filepath
if [ $? -ne 0 ]; then
    exit 1
fi

check_file $nav_controller_filepath
if [ $? -ne 0 ]; then
    exit 1
fi


# source all files in the sources array
for src in "${sources[@]}"
do
    p=$(realpath "$src")
    if [ $quiet -eq 0 ]; then
        echo "Sourcing $p"
    fi
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
            if [ $quiet -eq 0 ]; then
                echo "output.log is too big. Deleting..."
            fi
            rm output.log
        fi
    fi

    # start the experiment
    tmux new-session -d -s "$session_name" -x "$(tput cols)" -y "$(tput lines)" \; pipe-pane -o 'cat >>output.log'
    tmux rename-window -t 0 'window 0'

    sleep 1
    if [ $quiet -eq 0 ]; then
        echo "Start experiment" 
    fi

    # construct command to execute with tmux
    if [ $run_headless -eq 1 ]; then
        cmd='ros2 launch wpf_tools waypoint_follower.launch.py run_headless:=True waypoints_filepath:='$waypoints_filepath' gps_error_simulator_config_filepath:='$gnss_error_filepath' nav_planner_config_filepath:='$nav_planner_filepath' nav_controller_config_filepath:='$nav_controller_filepath
    else
        cmd='ros2 launch wpf_tools waypoint_follower.launch.py run_headless:=False waypoints_filepath:='$waypoints_filepath' gps_error_simulator_config_filepath:='$gnss_error_filepath' nav_planner_config_filepath:='$nav_planner_filepath' nav_controller_config_filepath:='$nav_controller_filepath
    fi

    if [ $quiet -eq 0 ]; then
        echo "Executing command: $cmd"
    fi

    # execute command in tmux
    tmux send-keys -t 'window 0' "$cmd" C-m

    start_time=$(date +%s)

    sleep 20
    # after 20 seconds everything should be running fine
    if [ -z "$(ros2 topic list | grep 'status/gazebo/OK')" ]; then
        echo "Gazebo did not start. Restarting..."
        tmux send-keys -t 'window 0' C-c
        sleep 10
        tmux kill-session -t "$session_name"
        sleep 5 # some nodes keep running for some time
        remove_results
        continue
    fi
    if [ -z "$(ros2 topic list | grep 'status/localization/OK')" ]; then
        echo "Localization is not working correctly. Restarting..."
        tmux send-keys -t 'window 0' C-c
        sleep 10
        tmux kill-session -t "$session_name"
        sleep 5 # some nodes keep running for some time
        remove_results
        continue
    fi
    if [ -z "$(ros2 topic list | grep 'status/navigation/OK')" ]; then
        echo "Navigation is not working correctly. Restarting..."
        tmux send-keys -t 'window 0' C-c
        sleep 10
        tmux kill-session -t "$session_name"
        sleep 5 # some nodes keep running for some time
        remove_results
        continue
    fi

    started=1
done

# wait for the experiment to finish
elapsed_time=$(($(date +%s) - start_time))
while [ $elapsed_time -lt $max_runtime ]; do
    if ! [ -z "$(ros2 topic list | grep 'status/goal_checker/OK')" ]; then
        if [ $quiet -eq 0 ]; then
            echo "Goal reached after $elapsed_time seconds"
            echo "Ending..."
        fi
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