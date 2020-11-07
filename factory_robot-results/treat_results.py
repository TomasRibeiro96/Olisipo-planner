import csv

number_machines = 3
dispatcher_list = ['esterel', 'adaptable']

# domain = 'factory_robot'
# problem_list = [i for i in range(1,11)]

domain = 'adv_factory_robot'
problem_list = [i for i in range(1,9)]

common_path = '/home/tomas/ros_ws/src/ROSPlan/src/rosplan/factory_robot-results/'+domain+'/'+'problem_m'+str(number_machines)+'/'

def calculateStandardDeviation(lst, avg, total_number):
    sum_dev = 0.0
    for item in lst:
        sum_dev += (float(item) - avg)**2

    # print '\tAverage: ', avg
    # print '\tTotal number: ', total_number
    # print '\tStandard Deviation: ', (sum_dev/(total_number-1))**0.5
    return ( sum_dev / (float(total_number)-1) )**0.5


if __name__ == "__main__":

    for dispatcher in dispatcher_list:
        results_file = open(common_path+'results_all-m'+str(number_machines)+'.csv', 'a')
        suc_results_file = open(common_path+'results_suc-m'+str(number_machines)+'.csv', 'a')
        fail_results_file = open(common_path+'results_fail-m'+str(number_machines)+'.csv', 'a')

        results_file.write(dispatcher+'\n')
        results_file.write('Problem, Total, Successes, Failures, Replans, Replans_std_dev, Actions, Actions_std_dev,\n')

        suc_results_file.write(dispatcher+'\n')
        suc_results_file.write('Problem, Replans, Replans_std_dev, Actions, Actions_std_dev, \n')

        fail_results_file.write(dispatcher+', \n')
        fail_results_file.write('Problem, Actions, Actions_std_dev, \n')

        replans_list = list()
        suc_replans_list = list()
        actions_list = list()
        suc_actions_list = list()
        fail_actions_list = list()

        for problem in problem_list:
            
            number_suc = 0
            number_fails = 0
            number_total = 0

            # There is no fail_avg_replans because it would always be the maximum number of replans (10)
            sum_replans = 0
            suc_sum_replans = 0
            
            sum_actions = 0
            suc_sum_actions = 0
            fail_sum_actions = 0

            file_name = common_path + 'exp_'+dispatcher+'-problem_m'+str(number_machines)+'-'+str(problem)+'.csv'
            file_csv = open(file_name, 'r')

            # Iterate over results files
            for line in file_csv:
                split_line = line.split(', ')
                
                success = int(split_line[0])
                replans = int(split_line[1])
                actions = int(split_line[2])

                number_total += 1
                sum_replans += replans
                sum_actions += actions

                replans_list.append(replans)
                actions_list.append(actions)

                if success == 1:
                    number_suc += 1
                    suc_sum_replans += replans
                    suc_sum_actions += actions
                    suc_actions_list.append(actions)
                    suc_replans_list.append(replans)
                else:
                    number_fails += 1
                    fail_sum_actions += actions
                    fail_actions_list.append(actions)
            
            avg_success = float(number_suc)/float(number_total)

            avg_replans = float(sum_replans)/float(number_total)
            # print 'Calculating std_dev for Replans: ', problem, ' | ', dispatcher
            std_dev_replans = calculateStandardDeviation(replans_list, avg_replans, number_total)

            avg_actions = float(sum_actions)/float(number_total)
            # print 'Calculating std_dev for Actions: ', problem, ' | ', dispatcher
            std_dev_actions = calculateStandardDeviation(actions_list, avg_actions, number_total)

            problem_name = 'm' + str(number_machines) + '_p' + str(problem)

            line_all = problem_name + ', ' + str(number_total) + ', ' + str(number_suc) + ', ' \
                        + str(number_fails) + ', ' + str(avg_replans) + ', ' + str(std_dev_replans) \
                        + ', ' + str(avg_actions) + ', ' + str(std_dev_actions) + ', \n'
            
            results_file.write(line_all)


            # If there are successes
            if number_suc > 1:
                print('Inside successes')
                print('Suc_sum_replans: ' + str(suc_sum_replans))
                suc_avg_replans = float(suc_sum_replans)/float(number_suc)
                print('Suc_avg_replans 1: ' + str(suc_avg_replans))
                # print 'Calculating std_dev for Replans of Successes: ', problem, ' | ', dispatcher
                suc_std_dev_replans = calculateStandardDeviation(suc_replans_list, suc_avg_replans, number_suc)

                suc_avg_actions = float(suc_sum_actions)/float(number_suc)
                # print 'Calculating std_dev for Actions of Successes: ', problem, ' | ', dispatcher
                suc_std_dev_actions = calculateStandardDeviation(suc_actions_list, suc_avg_actions, number_suc)

                print('Suc_avg_replans 2: ' + str(suc_avg_replans))
                line_suc = problem_name + ', ' + str(suc_avg_replans) + ', ' + str(suc_std_dev_replans) \
                            + ', ' + str(suc_avg_actions) + ', ' + str(suc_std_dev_actions) + ', ' + '\n'
                suc_results_file.write(line_suc)


            # If there are failures
            if number_fails > 1:
                fail_avg_actions = float(fail_sum_actions)/float(number_fails)
                # print 'Calculating std_dev for Actions of Failures: ', problem, ' | ', dispatcher
                fail_std_dev_actions = calculateStandardDeviation(fail_actions_list, fail_avg_actions, number_fails)

                line_fail = problem_name + ', ' + str(fail_avg_actions) + ', ' + str(fail_std_dev_actions) + ', ' + '\n'

                fail_results_file.write(line_fail)

        results_file.close()
        suc_results_file.close()
        fail_results_file.close()