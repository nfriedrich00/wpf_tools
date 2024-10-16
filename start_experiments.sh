#! /bin/bash

# run experiments for several times to get the average results

# parse arguments
loops=1
start_script="./start_single_experiment.sh"
results_dir="~/Documents/wpf/logs"
other_args=""

# Loop through arguments, separate -k and its value, store others in other_args
while [[ $# -gt 0 ]]; do
  case $1 in
    -k)
      # Shift to next argument (value)
      shift
      if [[ $# -gt 0 ]]; then
        loops="$1"
        shift
      else
        echo "Error: -k flag requires a value"
        exit 1
      fi
      ;;
    -r)
      # Shift to next argument (value)
      results_dir="$2"
      shift
      if [[ $# -gt 0 ]]; then
        results_dir="$1"
        shift
      else
        echo "Error: -r flag requires a value"
        exit 1
      fi
      ;;
    *)
      other_args="$other_args $1"
      shift
      ;;
  esac
done

# No need for additional shift

# pass remaining arguments (stored in other_args) to start_single_experiment.sh
for ((i=1; i<=$loops; i++)); do
  echo "Run experiment $i"
  # Use $other_args directly and -r $results_dir
  "$start_script" $other_args -r $results_dir

  # check return code to be 0 or 2
  if [ $? -ne 0 ] && [ $? -ne 2 ]; then
    # rename the results file to avoid overwriting
    mv $results_dir/results.yaml $results_dir/results_$i.yaml
  else
    echo "Experiment $i failed"
  fi
done
