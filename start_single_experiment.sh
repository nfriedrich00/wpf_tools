#! /bin/bash

export DISPLAY=:1

if [ $(command -v tmux) ]; then
echo "TMUX already installed"
else
    sudo apt update; 
    sudo apt install -y tmux;
fi 

SESSION="Experiment"
max_runtime=300
max_retries=20
waypoints_filepath="~/Documents/ros2/wpf_ws/src/wpf_tools/config/waypoints_cosine.yaml"
gnss_error_filepath="~/Documents/ros2/wpf_ws/src/wpf_tools/config/gps_error_simulator_config.yaml"
nav_planner_filepath="~/Documents/ros2/wpf_ws/src/wpf_tools/config/planner_straight_line.yaml"
nav_controller_filepath="~/Documents/ros2/wpf_ws/src/wpf_tools/config/controller_mppi.yaml"
results_dir="~/Documents/wpf/logs"
output_file="~/Documents/wpf/results.yaml"
sources=( "/opt/ros/iron/setup.bash" "~/Documents/ros2/dmc_11_ws/install/setup.bash" "~/Documents/ros2/wpf_ws/install/setup.bash" )
init_sources=0
run_headless=1
quiet=0
rm_results=1
logging_file="output.log"
record_rosbag_path=""
record_rosbag=0

function display_help {
    echo "Usage: $0 [-w waypoints_filepath] [-g gnss_error_filepath] [-p nav_planner_filepath] [-c nav_controller_filepath] [-t max_runtime] [-m max_retries] [-s source] [-r results_dir] [-o output_file] [-v run_headless] [-q quiet] [-z rm_results] [-l logging_file] [-b record_rosbag_path]"
    echo "Options:"
    echo "  -w waypoints_filepath: Path to the waypoints file"
    echo "  -g gnss_error_filepath: Path to the GNSS error file"
    echo "  -p nav_planner_filepath: Path to the navigation planner file"
    echo "  -c nav_controller_filepath: Path to the navigation controller file"
    echo "  -t max_runtime: Maximum runtime of the experiment in seconds"
    echo "  -m max_retries: Maximum number of retries to start the experiment"
    echo "  -s source: Source file to source before starting the experiment"
    echo "  -r results_dir: Directory to store the results"
    echo "  -o output_file: File to store the results"
    echo "  -v run_headless: Run the experiment headless (1) or with GUI (0)"
    echo "  -q quiet: Run the experiment in quiet mode (1) or verbose mode (0)"
    echo "  -z rm_results: Remove the results directory (1) or keep it (0)"
    echo "  -l logging_file: File to log the output of the experiment"
    echo "  -b record_rosbag_path: Path to record the rosbag"
}

# parse arguments to the script
# optionally accept a new path for the config_filepath, max_runtime, session_name and sources
while getopts "w:g:p:c:t:s:r:o:v:q:m:z:l:b:h" opt; do
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
        l )
            logging_file=$OPTARG
            ;;
        m ) 
            max_retries=$OPTARG
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
        z)
            # accept 0,1,true,false,True,False,TRUE,FALSE and convert to 0,1
            if [ $OPTARG == "0" ] || [ $OPTARG == "1" ] || [ $OPTARG == "true" ] || [ $OPTARG == "false" ] || [ $OPTARG == "True" ] || [ $OPTARG == "False" ] || [ $OPTARG == "TRUE" ] || [ $OPTARG == "FALSE" ]; then
                if [ $OPTARG == "0" ] || [ $OPTARG == "false" ] || [ $OPTARG == "False" ] || [ $OPTARG == "FALSE" ]; then
                    rm_results=0
                else
                    rm_results=1
                fi
            else
                echo "Invalid value for rm_results: $OPTARG"
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
        b)
            # where to store the rosbag
            record_rosbag_path=$OPTARG
            ;;

        h)
            display_help
            exit 0
            ;;
        \?)
            echo "Invalid option: $OPTARG" 1>&2
            display_help
            exit 1
            ;;
    esac
done

# define function to run evaluation
function analyze_data {
    if [ $quiet -eq 0 ]; then
        echo "Analyzing data..."
        ros2 run wpf_tools analyze_data --ros-args -p logs_path:=$results_dir -p results_path:=$output_file
    else
        ros2 run wpf_tools analyze_data --ros-args -p logs_path:=$results_dir -p results_path:=$output_file &> /dev/null
    fi
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
    if [ $rm_results -ne 0 ]; then
        if [ $quiet -eq 0 ]; then
            echo "Removing results..."
        fi
        rm -rf $results_dir/*
    fi
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

# function to kill unsucessfull tmux session
function kill_session {
    tmux send-keys -t "$1:0" C-c
    sleep 10
    tmux kill-session -t "$1"
    sleep 5 # some nodes keep running for some time
}

# function to kill all tmux sessions via killing the tmux server but sendint SIGINT to all tmux sessions
function kill_all_sessions {
    for s in $(tmux ls | cut -d: -f1); do
        # send SIGINT to all windows in the session
        tmux send-keys -t "$s:0" C-c
    done
    sleep 15
    tmux kill-server
    sleep 2
}

# set handler for SIGINT and kill all tmux sessions starting with $SESSION
trap 'echo "SIGINT detected in start_single_experiment.sh! Exiting..."; kill_all_sessions; exit 1' SIGINT

# resolve all file paths
waypoints_filepath=$(resolve_path $waypoints_filepath)
gnss_error_filepath=$(resolve_path $gnss_error_filepath)
nav_planner_filepath=$(resolve_path $nav_planner_filepath)
nav_controller_filepath=$(resolve_path $nav_controller_filepath)
results_dir=$(resolve_path $results_dir)
output_file=$(resolve_path $output_file)
record_rosbag_path=$(resolve_path $record_rosbag_path)

# print all arguments for debugging
if [ $quiet -eq 0 ]; then
    echo "waypoints_filepath: $waypoints_filepath"
    echo "gnss_error_filepath: $gnss_error_filepath"
    echo "nav_planner_filepath: $nav_planner_filepath"
    echo "nav_controller_filepath: $nav_controller_filepath"
    echo "results_dir: $results_dir"
    echo "output_file: $output_file"
    echo "max_runtime: $max_runtime"
    echo "max_retries: $max_retries"
    echo "run_headless: $run_headless"
    echo "quiet: $quiet"
    echo "rm_results: $rm_results"
    echo "logging_file: $logging_file"
    echo "record_rosbag_path: $record_rosbag_path"
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

# check whether results_dir exists
if [ -d $results_dir ]; then
    if [ $quiet -eq 0 ]; then
        echo "Results directory $results_dir found"
    fi
else
    echo "Results directory $results_dir not found"
    exit 1
fi

# check if rosabag path is not empty
if [ ! -z "$record_rosbag_path" ]; then
    if [ $quiet -eq 0 ]; then
        echo "Rosbag path set to $record_rosbag_path"
    fi

    # check if rosbag path exists and create it if it does not
    if [ ! -d "$record_rosbag_path" ]; then
        if [ $quiet -eq 0 ]; then
            echo "Creating rosbag path $record_rosbag_path"
        fi
        mkdir -p $record_rosbag_path
    fi

    # now rosbag path should exist
    if [ -d "$record_rosbag_path" ]; then
        record_rosbag=1

        # find a unique rosbag path
        i=0
        record_rosbag_path_base=$record_rosbag_path
        record_rosbag_path="$record_rosbag_path_base/bag-$i"
        while [[ -d "$record_rosbag_path" ]]; do
            ((i++))
            record_rosbag_path="$record_rosbag_path_base/bag-$i"
        done

        if [ $quiet -eq 0 ]; then
            echo "Unique rosbag path found: $record_rosbag_path"
        fi
    else 
        if [ $quiet -eq 0 ]; then
            echo "Rosbag path $record_rosbag_path does not exist or is not a directory"
        fi
    fi
fi


# source all files in the sources array
for src in "${sources[@]}"
do
    p=$(resolve_path "$src")
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
retries=0
kill=0

# remove results from previous runs
remove_results

# loop until we successfully start the experiment or we reach the maximum number of retries
while [ $started -eq 0 ] && [ $retries -lt $max_retries ]; do

    # find a unique session name
    while [[ $(tmux ls 2>&1 | grep -E "^$session_name:") ]]; do
        ((i++))
        session_name="$SESSION-$i"
    done

    if [ $quiet -eq 0 ]; then
        echo "Unique session name found: $session_name"
    fi

    # check size of output.log and delete if it is too big
    if [ -f $logging_file ]; then
        size=$(du -k $logging_file | cut -f1)
        if [ $size -gt 100000 ]; then
            if [ $quiet -eq 0 ]; then
                echo "$logging_file is too big. Deleting..."
            fi
            rm $logging_file
        fi
    fi

    # are we running in our own container? Check for /entrypoint.sh
    entrypoint_cmd=""
    if [ -f /entrypoint.sh ]; then
        if [ $quiet -eq 0 ]; then
            echo "Running in container - using entrypoint to setup environment"
        fi
        entrypoint_cmd="/entrypoint.sh"
    fi

    # construct command to execute with tmux
    headless_str="False"
    if [ $run_headless -eq 1 ]; then
        headless_str="True"
    fi 
    cmd="$entrypoint_cmd ros2 launch wpf_tools waypoint_follower.launch.py run_headless:=$headless_str logs_path:=$results_dir waypoints_filepath:=$waypoints_filepath gps_error_simulator_config_filepath:=$gnss_error_filepath nav_planner_config_filepath:=$nav_planner_filepath nav_controller_config_filepath:=$nav_controller_filepath"

    if [ $quiet -eq 0 ]; then
        echo "Executing command: $cmd"
    fi


    # start session for experiment
    tmux new-session -d -s "$session_name-experiment" -x "$(tput cols)" -y "$(tput lines)" \; pipe-pane -o 'cat >> '"$logging_file" \;
    sleep 1
    tmux send-keys -t "$session_name-experiment" "$cmd" C-m

    # start session for rosbag recording
    if [ $record_rosbag -eq 1 ]; then
        tmux new-session -d -s "$session_name-rosbag" \; pipe-pane -o 'cat >> '"$logging_file" \;
        sleep 1
        tmux send-keys -t "$session_name-rosbag" "ros2 bag record -a -o $record_rosbag_path" C-m
    fi

    sleep 1
    if [ $quiet -eq 0 ]; then
        echo "Start experiment - Retries: $retries, started: $started" 
    fi

    start_time=$(date +%s)
    ((retries++))
    sleep 20
    # after 20 seconds everything should be running fine
    if [ -z "$(ros2 topic list | grep 'status/gazebo/OK')" ]; then
        echo "Gazebo did not start. Restarting..."
        kill=1
    fi
    if [ -z "$(ros2 topic list | grep 'status/localization/OK')" ]; then
        echo "Localization is not working correctly. Restarting..."
        kill=1
    fi
    if [ -z "$(ros2 topic list | grep 'status/navigation/OK')" ]; then
        echo "Navigation is not working correctly. Restarting..."
        kill=1
    fi

    if [ $kill -eq 1 ]; then
        kill_all_sessions
        if [ $record_rosbag -eq 1 ]; then
            # remove rosbag file
            rm -rf $record_rosbag_path
        fi
        remove_results
        

        kill=0
        continue
    fi

    
    if [ $quiet -eq 0 ]; then
        echo "Started!"
    fi
    started=1
done

# exit if we did not manage to start simulation
if [ $started -eq 0 ]; then
    kill_all_sessions
    if [ $record_rosbag -eq 1 ]; then
        # remove rosbag file
        rm -rf $record_rosbag_path
    fi

    # analyze data
    remove_results
    exit 1
fi

# wait for the experiment to finish
elapsed_time=$(($(date +%s) - start_time))
printed_waiting=0
while [ $elapsed_time -lt $max_runtime ]; do
    if ! [ -z "$(ros2 topic list | grep 'status/goal_checker/OK')" ]; then
        if [ $quiet -eq 0 ]; then
            echo "Goal reached after $elapsed_time seconds"
            echo "Ending..."
        fi

        # send SIGINT to the tmux sessions
        tmux send-keys -t "$session_name-experiment" C-c
        sleep 10

        if [ $record_rosbag -eq 1 ]; then
            tmux send-keys -t "$session_name-rosbag" C-c
        fi
        
        sleep 5
        # wait for results to be written
        check_results

        # kill session
        kill_all_sessions
        sleep 5 # some nodes keep running for some time

        # analyze data
        analyze_data
        remove_results
        exit 0
    fi

    if [ $quiet -eq 0 ]; then
        if [ $printed_waiting -eq 0 ]; then 
            echo "Waiting for simulation..."
            printed_waiting=1
        fi
    fi
    sleep 1
    elapsed_time=$(($(date +%s) - start_time))
done

# timeout
echo "Timeout, killing..."

# send SIGINT to the tmux sessions
tmux send-keys -t "$session_name-experiment" C-c
sleep 10

if [ $record_rosbag -eq 1 ]; then
    tmux send-keys -t "$session_name-rosbag" C-c
fi

sleep 5
# wait for results to be written
check_results

# kill session
kill_all_sessions

# analyze data
analyze_data
remove_results
exit 2
