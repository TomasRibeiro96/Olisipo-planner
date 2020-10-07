#!/bin/bash

# Calls multiple experiments in parallel exploiting the ROS capability for
# using multiple PORTS, 

# NOTE: domain is same for all experiments: domain_robot_delivery.pddl


# arg 1: adaptable_plan_dispatcher_required options: (true, false)
# arg 2: category, temporal or non temporal? options: (iros_problems_free, iros_problems_deadlines)
# arg 3: problem instance name, options: (too large, check below for details)
function call_exec_experiment_with_wait(){
    PORT=11${i}11
    export ROS_MASTER_URI=http://localhost:$PORT # roscore -p $PORT not needed!
    # ./procs[${i}] &
    echo "roslaunch rosplan_csp_exec_demo rosplan_csp_exec_demo.launch adaptable_plan_dispatcher_required:=$1 category:=$2 domain:=$3 number_machines:=$4 probabilities_problem:=$5"
    timeout 600 roslaunch rosplan_csp_exec_demo rosplan_csp_exec_demo.launch adaptable_plan_dispatcher_required:=$1 category:=$2 domain:=$3 number_machines:=$4 probabilities_problem:=$5
    # run processes and store pids in array
    pids[${i}]=$!

    # wait for all pids to finish
    for pid in ${pids[*]};
        do
            wait $pid
        done

    echo '---------------- DONE WITH BATCH ----------------'
}


declare -a number_machines_vec=("3" "4")
declare -a number_machines=${number_machines_vec[0]}

declare -a number_runs=$((100))

declare -a exp_count=$((0))
declare -a total_count=$((2*7*$number_runs))


#### With esterel dispatcher ####
# Iterate over probabilities problem
for problem in {1..7}
do
    # Number of runs for each pair of problems and probabilities
    for k in $( eval echo {1..$number_runs} )
    do
        call_exec_experiment_with_wait false factory_robot_problems domain_simple_factory_robot $number_machines $problem
        exp_count=$(($exp_count + 1))
        echo ""
        echo "|||||||||||||||||||||||||||||||||| EXECUTED $exp_count OF $total_count ||||||||||||||||||||||||||||||||||"
        echo ""
    done
done

#### With adaptable dispatcher ####
# Iterate over probabilities problem
for problem in {1..7}
do
    # Number of runs for each pair of problems and probabilities
    for k in $( eval echo {1..$number_runs} )
    do
        call_exec_experiment_with_wait true factory_robot_problems domain_simple_factory_robot $number_machines $problem
        exp_count=$(($exp_count + 1))
        echo ""
        echo "|||||||||||||||||||||||||||||||||| EXECUTED $exp_count OF $total_count ||||||||||||||||||||||||||||||||||"
        echo ""
    done
done