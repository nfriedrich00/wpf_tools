#! /bin/bash

# run experiments for several times to get the average results

# parse arguments
loops=1
start_script="./start_single_experiment.sh"
other_args=""

while getopts ":k:" opt; do
  case $opt in
    k)
      loops="$OPTARG"
      ;;
  esac
done

# shift processed options (excluding the first argument, which is the script name)
shift $((OPTIND))

# store remaining arguments in a variable, excluding '-k'
other_args="$@"

# pass remaining arguments to start_single_experiment.sh
for ((i=1; i<=$loops; i++)); do
  echo "Run experiment $i"
  "$start_script" "$@"  # Double quotes preserve arguments as individual words
done