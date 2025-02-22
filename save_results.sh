#!/bin/bash

# Directory and file to monitor
WATCHED_FILE="/home/ubuntu/Documents/icra/logs/results.yaml"
LOGS_DIR="/home/ubuntu/Documents/icra/logs"
BACKUP_DIR="/home/ubuntu/Documents/icra/backup"

while true; do
    if [ -f "$WATCHED_FILE" ]; then
        TIMESTAMP=$(date +"%Y%m%d_%H%M%S")
        TARGET_DIR="$BACKUP_DIR/$TIMESTAMP"
        mv "$LOGS_DIR" "$TARGET_DIR"
        echo "Moved logs to $TARGET_DIR" >> /home/ubuntu/Documents/icra/log_monitor.log
    fi
    sleep 1
done
