#! /bin/bash

# run experiments for several times to get the average results

# parse arguments
loops=1
start_script="./start_single_experiment.sh"
results_file="~/Documents/wpf/results.yaml"
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
    -o)
      # Shift to next argument (value)
      results_file="$2"
      shift
      if [[ $# -gt 0 ]]; then
        results_file="$1"
        shift
      else
        echo "Error: -o flag requires a value"
        exit 1
      fi
      ;;
    *)
      other_args="$other_args $1"
      shift
      ;;
  esac
done

# function for replace ~ in string with $HOME
function resolve_path {
    echo $(eval echo $1)
}

results_file=$(resolve_path $results_file)

# pass remaining arguments (stored in other_args) to start_single_experiment.sh
for ((i=1; i<=$loops; i++)); do
  echo "Run experiment $i"
  # Use $other_args directly and -o $results_file
  "$start_script" $other_args -o $results_file
  res=$?
  echo "Script returned code: $res"

  # if return code is 0 or 2, move the results file to a new name
  if [ $res -eq 0 ] || [ $res -eq 2 ]; then
    
    # check if results file exists
    if [ -f $results_file ]; then
        # rename the results file to avoid overwriting
        filename=$(basename "$results_file" .yaml)
        filepath=$(dirname "$results_file")
        echo "Moving results from '$results_file' to '$filepath/$filename-$i.yaml'"
        mv $results_file "$filepath/$filename-$i.yaml"
    else
        echo "Results file '$results_file' not found"
    fi
  else
    echo "Experiment $i failed"
  fi
done
