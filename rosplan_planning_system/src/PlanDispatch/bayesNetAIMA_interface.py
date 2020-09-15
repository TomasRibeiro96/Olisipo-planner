import ast
import sys
# Adding path to aima folder so its files are accessible
sys.path.insert(1, '/home/tomas/ros_ws/src/ROSPlan/src/rosplan/rosplan_planning_system/src/PlanDispatch/aima-python')
import probability

list_actions_ = list()
list_goal_ = list()
layer_goal_ = '0'

def addNodeToEvidence(evidence, node):
    evidence[node] = True
    return evidence


def writeProbToFile(prob_list):
    file = open('prob_bayesNetAIMA.txt', 'w')
    for prob in prob_list:
        file.write(str(prob) + '\n')
    file.close()


def buildBayesNetwork():
    global list_actions_
    global layer_goal_

    ### Reading from file ###
    file = open('bayesNetAIMA.txt', 'r')
    line = file.readline()

    bayes_net_list = list()
    while line != '':
        # print('LINE: ' + str(line))
        line = file.readline()
        line = line.strip('\n')
        # print('Name: ' + str(line))
        name = line

        if len(name.split('$')) > 1:
            list_actions_.append(name)
        elif name.split('%')[1] == layer_goal_:
            list_goal_.append(name)
        else:
            layer_goal_ = name.split('%')[1]
            list_goal_.clear()
            list_goal_.append(name)

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

    return probability.BayesNet(bayes_net_list)


def main():
    # Skip door
    # list_actions = ['navigate_start#mbot#wp1#wp2#door1#door2$1', 'navigate_end#mbot#wp1#wp2#door1#door2$2',
    #                 'open_door_start#mbot#wp2#door2$3', 'open_door_end#mbot#wp2#door2$4', 'navigate_start#mbot#wp2#wp3#door2#door3$5',
    #                 'navigate_end#mbot#wp2#wp3#door2#door3$6']
    # list_goal = ['robot_at#mbot#wp3%6']

    # Factory robot
    # list_actions = ['navigate_start#mbot#m1#m2$1', 'navigate_end#mbot#m1#m2$2', 'fix_machine_start#mbot#m2$3',
    #                 'fix_machine_end#mbot#m2$4', 'navigate_start#mbot#m2#m3$5', 'navigate_end#mbot#m2#m3$6',
    #                 'fix_machine_start#mbot#m3$7', 'fix_machine_end#mbot#m3$8']
    # list_goal = ['machine_is_fixed#m1%8', 'machine_is_fixed#m2%8', 'machine_is_fixed#m3%8']
    

    bayes_net = buildBayesNetwork()

    evidence = {}

    prob_list = list()
    previous_prob = 1


    ### Calculating actions probability
    for node in list_actions_:
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
    
    ### Calculating goal probability
    goal_prob = 1
    for node in list_goal_:
        # print('>>> Goal: ' + node)

        # Using variable elimination
        prob = probability.elimination_ask(node, evidence, bayes_net).prob[True]

        goal_prob = goal_prob*prob
        
        addNodeToEvidence(evidence, node)

        # print('>>> Probability: ' + str(prob_list[-1]) + '\n')

    total_joint_prob = prob_list[-1]*goal_prob
    prob_list.append(total_joint_prob)

    print('Full joint probability: ' + str(total_joint_prob))

    
    writeProbToFile(prob_list)


if __name__ == "__main__":
    main()