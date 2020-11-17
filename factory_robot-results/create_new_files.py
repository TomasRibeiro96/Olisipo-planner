import csv

number_machines = 3
dispatcher_list = ['esterel', 'adaptable']

# domain = 'factory_robot'
# problem_list = [i for i in range(1,11)]

domain = 'adv_factory_robot'
problem_list = [i for i in range(1,9)]

data_path = '/home/tomas/ros_ws/src/ROSPlan/src/rosplan/factory_robot-results/'+domain+'/'+'problem_m'+str(number_machines)+'/'
common_path = '/home/tomas/ros_ws/src/ROSPlan/src/rosplan/factory_robot-results/'+domain+'/'+'problem_m'+str(number_machines)+'/treated_results/'


if __name__ == "__main__":


    for dispatcher in dispatcher_list:
        for problem in problem_list:

            file_name = data_path + 'exp_'+dispatcher+'-problem_m'+str(number_machines)+'-'+str(problem)+'.csv'
            file_csv = open(file_name, 'r')

            suc_results_file = open(common_path+'suc-'+dispatcher+'-m'+str(number_machines)+'_p'+str(problem)+'.csv', 'w')
            fail_results_file = open(common_path+'fail-'+dispatcher+'-m'+str(number_machines)+'_p'+str(problem)+'.csv', 'w')

            # Iterate over results files
            for line in file_csv:
                split_line = line.split(', ')

                if split_line[0] == '1':
                    suc_results_file.write(line)
                else:
                    fail_results_file.write(line)

            suc_results_file.close()
            fail_results_file.close()


            
# def writeResultsToFile(suc_replans, suc_actions, fail_actions):
#     suc_replans_file = open(common_path+'suc-replans.csv', 'w')
#     suc_actions_file = open(common_path+'suc-actions.csv', 'w')
#     fail_actions_file = open(common_path+'fail-actions.csv', 'w')

#     zipped_suc_replans = zip(*suc_replans)
#     zipped_suc_actions = zip(*suc_actions)
#     zipped_failed_actions = zip(*fail_actions)

#     suc_replans_writer = csv.writer(suc_replans_file, delimiter=',')
#     suc_actions_writer = csv.writer(suc_actions_file, delimiter=',')
#     fail_actions_writer = csv.writer(fail_actions_file, delimiter=',')

#     suc_replans_writer.writerows(zipped_suc_replans)
#     suc_actions_writer.writerows(zipped_suc_actions)
#     fail_actions_writer.writerows(zipped_failed_actions)

#     suc_replans_file.close()
#     suc_actions_file.close()
#     fail_actions_file.close()


# if __name__ == "__main__":

#     # suc_replans: [replans of esterel p1, replans of adaptable p1, replans of esterel p2, ...]
#     suc_replans = list()
#     # suc_actions: [actions of esterel p1, actions of adaptable p1, actions of esterel p2, ...]
#     suc_actions = list()
#     fail_actions = list()

#     for problem in problem_list:

#         for dispatcher in dispatcher_list:

#             suc_replans.append(list())
#             suc_actions.append(list())
#             fail_actions.append(list())

#             file_name = common_path + 'exp_'+dispatcher+'-problem_m'+str(number_machines)+'-'+str(problem)+'.csv'
#             file_csv = open(file_name, 'r')

#             # suc_results_file = open(common_path+'suc-'+dispatcher+'-m'+str(number_machines)+'_p'+str(problem)+'.csv', 'w')
#             # fail_results_file = open(common_path+'fail-'+dispatcher+'-m'+str(number_machines)+'_p'+str(problem)+'.csv', 'w')

#             # Iterate over results files
#             for line in file_csv:
#                 split_line = line.split(', ')

#                 success = split_line[0]
#                 replans = split_line[1]
#                 actions = split_line[2]

#                 if success:
#                     suc_replans[-1].append(replans)
#                     suc_actions[-1].append(actions)
#                     # suc_results_file.write(line)
#                 else:
#                     fail_actions[-1].append(actions)
#                     # fail_results_file.write(line)

#     writeResultsToFile(suc_replans, suc_actions, fail_actions)

#             # suc_results_file.close()
#             # fail_results_file.close()