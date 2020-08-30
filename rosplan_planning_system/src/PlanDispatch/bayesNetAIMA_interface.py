import ast
import sys
# Adding path to aima folder so its files are accessible
sys.path.insert(1, '/home/tomas/ros_ws/src/ROSPlan/src/rosplan/rosplan_planning_system/src/PlanDispatch/aima-python')
import probability


def addNodeToEvidence(evidence, node):
    evidence[node] = True
    return evidence


def writeProbToFile(prob_list):
    file = open('prob_bayesNetAIMA.txt', 'w')
    for prob in prob_list:
        file.write(str(prob) + '\n')
    file.close()


def main():
    list_actions = ['navigate_start#mbot#wp1#wp2#door1#door2$1', 'navigate_end#mbot#wp1#wp2#door1#door2$2', 'open_door_start#mbot#wp2#door2$3', 'open_door_end#mbot#wp2#door2$4', 'navigate_start#mbot#wp2#wp3#door2#door3$5', 'navigate_end#mbot#wp2#wp3#door2#door3$6']

    list_goal = ['robot_at#mbot#wp3%6']

    list_actions_goal = list_actions + list_goal

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


    for node in list_actions_goal:
        print('>>> Node: ' + node)

        # Using variable elimination
        prob = probability.elimination_ask(node, evidence, bayes_net).prob[True]

        if len(prob_list) > 0:
            joint_prob = prob*prob_list[-1]
        else:
            joint_prob = prob
            
        prob_list.append(joint_prob)
        
        addNodeToEvidence(evidence, node)

        print('>>> Probability: ' + str(prob_list[-1]) + '\n')
    
    writeProbToFile(prob_list)


if __name__ == "__main__":
    main()