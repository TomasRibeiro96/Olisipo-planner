#!/bin/bash

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


declare -a category="factory_robot_problems"
declare -a domain="domain_simple_factory_robot"
declare -a start_number_problems=$((13))
declare -a end_number_problems=$((16))

# declare -a category="adv_factory_robot_problems"
# declare -a domain="domain_adv_factory_robot"
# declare -a start_number_problems=$((2))
# declare -a end_number_problems=$((8))


declare -a number_machines=$((3))
declare -a number_runs=$((1000))

declare -a exp_count=$((0))
declare -a number_problems=$(($end_number_problems-$start_number_problems+1))
declare -a total_count=$((2*$number_problems*$number_runs))

#### With esterel dispatcher ####
# Iterate over probabilities problem
# for problem in {11..16}
# do
#     # Number of runs for each pair of problems and probabilities
#     for k in $( eval echo {1..$number_runs} )
#     do
#         call_exec_experiment_with_wait false $category $domain $number_machines $problem
#         exp_count=$(($exp_count + 1))
#         echo ""
#         echo "|||||||||||||||||||||||||||||||||| EXECUTED $exp_count OF $total_count ||||||||||||||||||||||||||||||||||"
#         echo ""
#     done
# done

#### With adaptable dispatcher ####
# Iterate over probabilities problem
# for problem in {11..16}
# do
#     # Number of runs for each pair of problems and probabilities
#     for k in $( eval echo {1..$number_runs} )
#     do
#         call_exec_experiment_with_wait true $category $domain $number_machines $problem
#         exp_count=$(($exp_count + 1))
#         echo ""
#         echo "|||||||||||||||||||||||||||||||||| EXECUTED $exp_count OF $total_count ||||||||||||||||||||||||||||||||||"
#         echo ""
#     done
# done

for problem in {13..16}
do
    # Number of runs for each pair of problems and probabilities
    for k in $( eval echo {1..$number_runs} )
    do
        #### With esterel dispatcher ####
        call_exec_experiment_with_wait false $category $domain $number_machines $problem
        exp_count=$(($exp_count + 1))
        echo ""
        echo "|||||||||||||||||||||||||||||||||| EXECUTED $exp_count OF $total_count ||||||||||||||||||||||||||||||||||"
        echo ""

        #### With adaptable dispatcher ####
        call_exec_experiment_with_wait true $category $domain $number_machines $problem
        exp_count=$(($exp_count + 1))
        echo ""
        echo "|||||||||||||||||||||||||||||||||| EXECUTED $exp_count OF $total_count ||||||||||||||||||||||||||||||||||"
        echo ""
    done
done