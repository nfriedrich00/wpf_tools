#!/bin/bash

# Directory and file to monitor
WATCHED_FILE="/home/ubuntu/Documents/icra/logs/results.yaml"
LOGS_DIR="/home/ubuntu/Documents/icra/logs"
BACKUP_DIR="/home/ubuntu/Documents/icra/backup"

if [ ! -d "$BACKUP_DIR" ]; then
    mkdir -p "$BACKUP_DIR"
fi

while true; do
    if [ -f "$WATCHED_FILE" ]; then
        TIMESTAMP=$(date +"%Y%m%d_%H%M%S")
        TARGET_DIR="$BACKUP_DIR/$TIMESTAMP"
        mv "$LOGS_DIR" "$TARGET_DIR"
        echo "Moved logs to $TARGET_DIR"
    fi
    sleep 1
done
