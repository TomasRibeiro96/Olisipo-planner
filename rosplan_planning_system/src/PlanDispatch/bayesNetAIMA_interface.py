import ast
import sys
# Adding path to aima folder so its files are accessible
sys.path.insert(1, '/home/tomas/ros_ws/src/ROSPlan/src/rosplan/rosplan_planning_system/src/PlanDispatch/aima-python')
import probability


def addNodeToEvidence(evidence, action):
    evidence[action] = True
    return evidence


def writeProbToFile(prob_list):
    file = open('prob_bayesNetAIMA.txt', 'w')
    for prob in prob_list:
        file.write(str(prob) + '\n')
    file.close()


def main():
    list_actions = ['navigate_start#mbot#wp1#wp2#door1#door2$1', 'navigate_end#mbot#wp1#wp2#door1#door2$2', 'open_door_start#mbot#wp2#door2$3', 'open_door_end#mbot#wp2#door2$4', 'navigate_start#mbot#wp2#wp3#door2#door3$5', 'navigate_end#mbot#wp2#wp3#door2#door3$6']

    file = open('bayesNetAIMA.txt', 'r')
    line = file.readline()

    bayes_net_list = list()

    while line != '':
        # print('LINE: ' + str(line))
        line = file.readline()
        line = line.strip('\n')
        # print('Name: ' + str(line))
        name = line

        line = file.readline()
        line = line.strip('\n')
        # print('Parents: ' + str(line))
        parents = ast.literal_eval(line)

        line = file.readline()
        line = line.strip('\n')
        # print('CPT: ' + str(line))
        cpt = ast.literal_eval(line)
        
        line = file.readline()
        bayes_net_list.append((name, parents, cpt))

    bayes_net = probability.BayesNet(bayes_net_list)

    evidence = {}

    prob_list = list()
    previous_prob = 1

    # # Just to check
    # node = bayes_net.variable_node('navigate_start#mbot#wp1#wp2#door1#door2$1')
    # print('>>> CPT: ' + str(node.cpt))
    # print('>>> Parents: ' + str(node.parents))
    # print('>>> Variable: ' + str(node.variable))
    # print('')
    # ####

    for action in list_actions:
        print('>>> Action: ' + action)
        # # Just to check
        # print('>>> CPT: ' + str(node.cpt))
        # print('>>> Parents: ' + str(node.parents))
        # print('>>> Variable: ' + str(node.variable))
        # ####

        # # Using variable elimination
        prob = probability.elimination_ask(action, evidence, bayes_net).prob[True]

        if len(prob_list) > 0:
            joint_prob = prob*prob_list[-1]
        else:
            joint_prob = prob
            
        prob_list.append(joint_prob)
        
        addNodeToEvidence(evidence, action)

        print('>>> Probability: ' + str(prob_list[-1]))

        print('')
    
    writeProbToFile(prob_list)


if __name__ == "__main__":
    main()