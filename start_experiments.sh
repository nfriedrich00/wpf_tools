#! /bin/bash

SESSION="Experiment"
max_runtime=300
config_filepath=~/wpf_ws/install/wpf_tools/share/wpf_tools/config/waypoint_follower_config.yaml


while true; do
    source /opt/ros/iron/setup.bash
    source ~/Documents/ros2/dmc_11_ws/install/setup.bash    # claudi
    source ~/wpf_ws/install/setup.bash                      # analyze tool
    source ~/planner_ws/install/setup.bash                  # straightline_planner


    tmux new-session -d -s "$SESSION" -d -x "$(tput cols)" -y "$(tput lines)"

    tmux rename-window -t 0 'window 0'
    echo "Start experiment" 
    tmux send-keys -t 'window 0' 'ros2 launch wpf_tools waypoint_follower.launch.py' C-m
    start_time=$(date +%s)

    sleep 20
    # after 20 seconds everything should be running fine

    if [ -z "$(ros2 topic list | grep 'status/gazebo/OK')" ]; then
        echo "Gazebo did not start. Restarting..."
        tmux send-keys -t 'window 0' C-c
        sleep 10
        tmux kill-session -t "$SESSION"
        sleep 5 # some nodes keep running for some time
        continue
    elif [ -z "$(ros2 topic list | grep 'status/localization/OK')" ]; then
        echo "Localization is not working correctly. Restarting..."
        tmux send-keys -t 'window 0' C-c
        sleep 10
        tmux kill-session -t "$SESSION"
        sleep 5 # some nodes keep running for some time
        continue
    elif [ -z "$(ros2 topic list | grep 'status/navigation/OK')" ]; then
        echo "Navigation is not working correctly. Restarting..."
        tmux send-keys -t 'window 0' C-c
        sleep 10
        tmux kill-session -t "$SESSION"
        sleep 5 # some nodes keep running for some time
        continue
    fi

    elapsed_time=$(($(date +%s) - start_time))
    while [ $elapsed_time -lt $max_runtime ]; do
        if ! [ -z "$(ros2 topic list | grep 'status/goal_checker/OK')" ]; then
            echo "Goal reached after $elapsed_time seconds"
            #####
            #
            # Change config file/parameters?
            #
            #####
            echo "Restarting..."
            tmux send-keys -t 'window 0' C-c
            sleep 10
            tmux kill-session -t "$SESSION"
            sleep 5 # some nodes keep running for some time
            break
        fi
        elapsed_time=$(($(date +%s) - start_time))
    done

    echo "Timeout, restarting..."
    tmux send-keys -t 'window 0' C-c
    sleep 10
    tmux kill-session -t "$SESSION"
    sleep 5 # some nodes keep running for some time
    continue
done