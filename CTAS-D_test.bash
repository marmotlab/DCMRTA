 #!/bin/bash
# pkill -f ./build/main

set -o monitor 
# means: run background processes in a separate processes...

todo_array=($(find . -wholename "./testSet_20A_50T_CONDET/env_*/planner_param.yaml")) # places output into an array
index=0
max_jobs=4

function add_next_job {
    # if still jobs to do then add one
    if [[ $index -lt ${#todo_array[*]} ]]
    # apparently stackoverflow doesn't like bash syntax
    # the hash in the if is not a comment - rather it's bash awkward way of getting its length
    then
        echo adding job ${todo_array[$index]}
        do_job ${todo_array[$index]} & 
        # replace the line above with the command you want
        index=$(($index+1))
    fi
}

function do_job {
  echo "Processing $1 file..."
  # take action on each file. $f store current file name
  results="$(echo $1 |sed -e 's/planner_param/results/')"
  ./baselines/CTAS-D/build/main "$1" "$results"
  sleep 2
}

trap add_next_job CHLD 
# execute add_next_job when we receive a child complete signal


# add initial set of jobs
while [[ $index -lt $max_jobs ]]
do
    add_next_job
done

# # wait for all jobs to complete
wait
echo "done"