count=$(ps aux | grep gazebo | grep -v grep | wc -l)
if [ "$count" -eq 1 ]; then
    pid=$(ps aux | grep gazebo | grep -v grep | awk '{print $2}')
    kill -9 $pid
fi

