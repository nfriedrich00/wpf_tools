#! /bin/bash

# run experiments for several times to get the average results

# parse arguments, here we only need -k to specify the number of loops, other arguments will be stored in other_args
loops=1
other_args=""
while getopts "k:" opt; do
  case $opt in
    k) loops=$OPTARG
    continue
    ;;
    \?) echo "Invalid option -$OPTARG" >&2
    ;;
  esac
  other_args="$other_args $OPTARG"
done

# run experiments
for ((i=1; i<=$loops; i++))
do
  echo "Run experiment $i"
  # parse all arguments (execpt k) to start_single_experiment.sh
  ./start_single_experiment.sh $@
done