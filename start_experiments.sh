#! /bin/bash

# run experiments for several times to get the average results

loops=10

# parse arguments with getopts
while getopts "k:" opt; do
  case $opt in
    k)
      echo "k = $OPTARG"
      loops=$OPTARG
      ;;
  esac
done

# run experiments
for ((i=1; i<=$loops; i++))
do
  echo "Run experiment $i"
  # parse all unread arguments to start_single_experiment.sh
  ./start_single_experiment.sh $@
done