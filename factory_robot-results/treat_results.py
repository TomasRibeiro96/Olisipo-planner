import csv

number_machines_list = [3]
dispatcher_list = ['esterel', 'adaptable']
problem_list = [i for i in range(1,6)]

common_path = '/home/tomas/ros_ws/src/ROSPlan/src/rosplan/factory_robot-results/adv_factory_robot/'

for number_machines in number_machines_list:
    problem_common_path = common_path + 'problem_m' + str(number_machines) + '/'

    for dispatcher in dispatcher_list:
        results_file = open(problem_common_path+'results_all-m'+str(number_machines)+'.csv', 'a')
        suc_results_file = open(problem_common_path+'results_suc-m'+str(number_machines)+'.csv', 'a')
        fail_results_file = open(problem_common_path+'results_fail-m'+str(number_machines)+'.csv', 'a')

        results_file.write('Problem, Success, Replans, Number Actions, Count\n')
        results_file.write(dispatcher+'\n')

        suc_results_file.write(dispatcher+', \n')
        suc_results_file.write('Problem, Replans, Number Actions, Count')

        fail_results_file.write(dispatcher+', \n')
        fail_results_file.write('Problem, Number Actions, Count')

        for problem in problem_list:
            avg_success = 0.0
            avg_replans = 0.0
            avg_replans_suc = 0.0
            avg_replans_fail = 0.0
            avg_actions = 0.0
            avg_actions_suc = 0.0
            avg_actions_fail = 0.0
            number_exp = 0
            suc_count = 0
            fail_count = 0
            exp_count = 0

            file_name = problem_common_path + 'exp_'+dispatcher+'-problem_m'+str(number_machines)+'-p'+str(problem)+'.csv'
            file_csv = open(file_name, 'r')

            for line in file_csv:
                split_line = line.split(', ')
                
                success = float(split_line[0])
                replans = float(split_line[1])
                actions = float(split_line[2])

                avg_success += success
                avg_replans += replans
                avg_actions += actions
                if success == 1.0:
                    avg_replans_suc += replans
                    avg_actions_suc += actions
                    suc_count += 1
                else:
                    avg_replans_fail += replans
                    avg_actions_fail += actions
                    fail_count += 1
                exp_count += 1
            
            start = 'm' + str(number_machines) + '_p' + str(problem) + ', '
            line_all = start+str(avg_success/exp_count)+', '+str(avg_replans/exp_count)+', '+str(avg_actions/exp_count)+', '+str(exp_count)+'\n'
            results_file.write(line_all)

            if not suc_count == 0:
                line_suc = start+str(avg_replans_suc/suc_count)+', '+str(avg_actions_suc/suc_count)+', '+str(suc_count)+'\n'
                suc_results_file.write(line_suc)

            if not fail_count == 0:
                line_fail = start+str(avg_replans_fail/fail_count)+', '+str(avg_actions_fail/fail_count)+', '+str(fail_count)+'\n'
                fail_results_file.write(line_fail)

        results_file.close()
        suc_results_file.close()
        fail_results_file.close()