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

    # dispatcher = 'esterel'
    dispatcher = 'adaptable'

    problem = 8

    file_name = common_path + 'exp_'+dispatcher+'-problem_m'+str(number_machines)+'-'+str(problem)+'.csv'
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