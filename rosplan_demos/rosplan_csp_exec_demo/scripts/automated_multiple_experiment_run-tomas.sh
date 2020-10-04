#!/bin/bash

# Calls multiple experiments in parallel exploiting the ROS capability for
# using multiple PORTS, 

# NOTE: domain is same for all experiments: domain_robot_delivery.pddl

# function call_exec_experiment(){
#     # make experiments in parallel
#     for i in `seq 0 9`; # do not set more than 9!
#         do
#             PORT=11${i}11
#             export ROS_MASTER_URI=http://localhost:$PORT # roscore -p $PORT not needed!
#             echo "roslaunch rosplan_csp_exec_demo rosplan_csp_exec_demo.launch adaptable_plan_dispatcher_required:=$1 category:=$2 problem:=$3 domain:=$4"
#             timeout 600 roslaunch rosplan_csp_exec_demo rosplan_csp_exec_demo.launch adaptable_plan_dispatcher_required:=$1 category:=$2 problem:=$3 domain:=$4&
#         done
# }


# arg 1: adaptable_plan_dispatcher_required options: (true, false)
# arg 2: category, temporal or non temporal? options: (iros_problems_free, iros_problems_deadlines)
# arg 3: problem instance name, options: (too large, check below for details)
function call_exec_experiment_with_wait(){
    for i in `seq 0 0`;
        do
            PORT=11${i}11
            export ROS_MASTER_URI=http://localhost:$PORT # roscore -p $PORT not needed!
            # ./procs[${i}] &
            echo "roslaunch rosplan_csp_exec_demo rosplan_csp_exec_demo.launch adaptable_plan_dispatcher_required:=$1 category:=$2 problem:=$3 domain:=$4 probabilities:=$5"
            timeout 600 roslaunch rosplan_csp_exec_demo rosplan_csp_exec_demo.launch adaptable_plan_dispatcher_required:=$1 category:=$2 problem:=$3 domain:=$4 probabilities:=$5&
            # run processes and store pids in array
            pids[${i}]=$!
        done

    # wait for all pids to finish
    for pid in ${pids[*]};
        do
            wait $pid
        done

    echo '---------------- DONE WITH BATCH ----------------'
}


# Factory robot problems (1)
declare -a factory_problems=("problem_m3")

# Factory robot probabilities (6)
declare -a factory_probabilities=("probabilities_m3_p1.txt" "probabilities_m3_p2.txt" "probabilities_m3_p3.txt" "probabilities_m3_p4.txt" "probabilities_m3_p5.txt" "probabilities_m3_p6.txt")

# echo "${nt_prob[1]}" # test

#### With esterel dispatcher ####
# Iterate over problems
for j in `seq 0 0`; do
    # Iterate over probabilities
    for i in `seq 0 5`; do
        # Number of runs for each pair of problems and probabilities
        for k in `seq 0 50`; do
            call_exec_experiment_with_wait false factory_robot_problems "${factory_problems[$j]}" domain_factory_robot "${factory_probabilities[$i]}"
        done
    done
done

#### With adaptable dispatcher ####
# Iterate over problems
for j in `seq 0 0`; do
    # Iterate over probabilities
    for i in `seq 0 5`; do
        # Number of runs for each pair of problems and probabilities
        for k in `seq 0 50`; do
            call_exec_experiment_with_wait true factory_robot_problems "${factory_problems[$j]}" domain_factory_robot "${factory_probabilities[$i]}"
        done
    done
done
