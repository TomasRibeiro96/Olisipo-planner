import csv

number_machines_list = [3]
dispatcher_list = ['esterel', 'adaptable']
problem_list = [i for i in range(1,19)]
domain = 'adv_factory_robot'
# domain = 'simple_factory_robot'

common_path = '/home/tomas/ros_ws/src/ROSPlan/src/rosplan/factory_robot-results/'+domain+'/'

def calculateStandardDeviation(lst, avg, total_number):
    sum_dev = 0.0
    for item in lst:
        sum_dev += (item - avg)**2

    # print '\tAverage: ', avg
    # print '\tTotal number: ', total_number
    # print '\tStandard Deviation: ', (sum_dev/(total_number-1))**0.5
    return (sum_dev/(total_number-1))**0.5


for number_machines in number_machines_list:
    problem_common_path = common_path + 'problem_m' + str(number_machines) + '/'

    for dispatcher in dispatcher_list:
        results_file = open(problem_common_path+'results_all-m'+str(number_machines)+'.csv', 'a')
        suc_results_file = open(problem_common_path+'results_suc-m'+str(number_machines)+'.csv', 'a')
        fail_results_file = open(problem_common_path+'results_fail-m'+str(number_machines)+'.csv', 'a')

        results_file.write(dispatcher+'\n')
        results_file.write('Problem, Success, Replans, Replans_std_dev, Actions, Actions_std_dev,\n')

        suc_results_file.write(dispatcher+'\n')
        suc_results_file.write('Problem, Replans, Replans_std_dev, Actions, Actions_std_dev, \n')

        fail_results_file.write(dispatcher+', \n')
        fail_results_file.write('Problem, Replans, Replans_std_dev, Actions, Actions_std_dev, \n')

        replans_list = list()
        suc_replans_list = list()
        fail_replans_list = list()
        actions_list = list()
        suc_actions_list = list()
        fail_actions_list = list()

        for problem in problem_list:
            avg_success = 0.0
            avg_replans = 0.0
            suc_avg_replans = 0.0
            fail_avg_replans = 0.0
            avg_actions = 0.0
            suc_avg_actions = 0.0
            fail_avg_actions = 0.0
            number_exp = 0
            suc_count = 0
            fail_count = 0
            exp_count = 0

            file_name = problem_common_path + 'exp_'+dispatcher+'-problem_m'+str(number_machines)+'-'+str(problem)+'.csv'
            file_csv = open(file_name, 'r')

            for line in file_csv:
                split_line = line.split(', ')
                
                success = float(split_line[0])
                replans = float(split_line[1])
                actions = float(split_line[2])

                avg_success += success
                avg_replans += replans
                avg_actions += actions

                replans_list.append(replans)
                actions_list.append(actions)

                if success == 1.0:
                    suc_avg_replans += replans
                    suc_avg_actions += actions
                    suc_actions_list.append(actions)
                    suc_replans_list.append(replans)
                    suc_count += 1
                else:
                    fail_avg_replans += replans
                    fail_avg_actions += actions
                    fail_actions_list.append(actions)
                    fail_replans_list.append(replans)
                    fail_count += 1
                exp_count += 1
            
            avg_success = avg_success/exp_count

            avg_replans = avg_replans/exp_count
            # print 'Calculating std_dev for Replans: ', problem, ' | ', dispatcher
            std_dev_replans = calculateStandardDeviation(replans_list, avg_replans, exp_count)

            avg_actions = avg_actions/exp_count
            # print 'Calculating std_dev for Actions: ', problem, ' | ', dispatcher
            std_dev_actions = calculateStandardDeviation(actions_list, avg_actions, exp_count)

            start = 'm' + str(number_machines) + '_p' + str(problem) + ', '
            line_all = start+str(avg_success)+', '+str(avg_replans)+', '+str(std_dev_replans)+', '+str(avg_actions)+', '+str(std_dev_actions)+', ''\n'
            results_file.write(line_all)

            if suc_count > 1:
                suc_avg_replans = suc_avg_replans/suc_count
                # print 'Calculating std_dev for Replans of Successes: ', problem, ' | ', dispatcher
                suc_std_dev_replans = calculateStandardDeviation(suc_replans_list, suc_avg_replans, suc_count)

                suc_avg_actions = suc_avg_actions/suc_count
                # print 'Calculating std_dev for Actions of Successes: ', problem, ' | ', dispatcher
                suc_std_dev_actions = calculateStandardDeviation(suc_actions_list, suc_avg_actions, suc_count)

                line_suc = start+str(suc_avg_replans)+', '+str(suc_std_dev_replans)+', '+str(suc_avg_actions)+', '+str(suc_std_dev_actions)+', '+'\n'
                suc_results_file.write(line_suc)

            if fail_count > 1:
                fail_avg_replans = fail_avg_replans/fail_count
                # print 'Calculating std_dev for Replans of Failures: ', problem, ' | ', dispatcher
                fail_std_dev_replans = calculateStandardDeviation(fail_replans_list, fail_avg_replans, fail_count)

                fail_avg_actions = fail_avg_actions/fail_count
                # print 'Calculating std_dev for Actions of Failures: ', problem, ' | ', dispatcher
                fail_std_dev_actions = calculateStandardDeviation(fail_actions_list, fail_avg_actions, fail_count)

                line_fail = start+str(fail_avg_replans)+', '+str(fail_std_dev_replans)+', '+str(fail_avg_actions)+', '+str(fail_std_dev_actions)+', '+'\n'
                fail_results_file.write(line_fail)

        results_file.close()
        suc_results_file.close()
        fail_results_file.close()