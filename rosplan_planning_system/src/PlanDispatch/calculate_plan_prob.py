#!/usr/bin/env python

import sys
import rospy
from rosplan_knowledge_msgs.srv import *
from rosplan_knowledge_msgs.msg import *
from rosplan_dispatch_msgs.msg import EsterelPlanArray
from std_msgs.msg import String
import matplotlib
import matplotlib.pyplot as plt
from numpy import *
from pomegranate import *
import collections
import time
import itertools

# pred_probabilities_map_[predicate] = [spont_false_true, spont_true_false]
pred_probabilities_map_ = dict()
# action_probabilities_map_[action] = [action_success, effects_success]
# effects_success[predicate_without_parameters] = probability
action_probabilities_map_ = dict()
cpds_map_ = dict()
all_nodes_ = list()
goal_ = set()
initial_state_ = set()

rospy.init_node('bayesian_network_calculator')

# TODO: Prunning could be done at the same time as CPDs are defined
#       Didn't do it so the code is clearer


class Action:

    actionsList = list()

    def __init__(self, name, duration = 0):
        if not isinstance(name, str):
            raise TypeError("Action name not a String")
        self.name = name
        self.duration = duration

        self.parameters = list()
        operator_details = rospy.ServiceProxy('/rosplan_knowledge_base/domain/operator_details', GetDomainOperatorDetailsService)(name)
        # rospy.loginfo('Parameters: ' + str(operator_details))
        number_param = len(operator_details.op.formula.typed_parameters)
        for i in range(number_param):
            self.parameters.append(operator_details.op.formula.typed_parameters[i].key)

        ### Getting effects and conditions from operator_details ###
        # I couldn't use a cycle because the parameter name of operator_details changes
        self.effects = dict()
        self.conditions = dict()
        self.comparison = dict()

        # at_start_add_effects
        self.effects['at_start_add_effects'] = list()
        if operator_details.op.at_start_add_effects:
            number_pred = len(operator_details.op.at_start_add_effects)
            for i in range(number_pred):
                self.effects['at_start_add_effects'].append(list())
                predicate_name = operator_details.op.at_start_add_effects[i].name
                self.effects['at_start_add_effects'][i].append(predicate_name)
                number_param = len(operator_details.op.at_start_add_effects[i].typed_parameters)
                for j in range(number_param):
                    variable = operator_details.op.at_start_add_effects[i].typed_parameters[j].key
                    self.effects['at_start_add_effects'][i].append(variable)
        
        # at_start_del_effects
        self.effects['at_start_del_effects'] = list()
        if operator_details.op.at_start_del_effects:
            number_pred = len(operator_details.op.at_start_del_effects)
            for i in range(number_pred):
                self.effects['at_start_del_effects'].append(list())
                predicate_name = operator_details.op.at_start_del_effects[i].name
                self.effects['at_start_del_effects'][i].append(predicate_name)
                number_param = len(operator_details.op.at_start_del_effects[i].typed_parameters)
                for j in range(number_param):
                    variable = operator_details.op.at_start_del_effects[i].typed_parameters[j].key
                    self.effects['at_start_del_effects'][i].append(variable)

        # at_end_add_effects
        self.effects['at_end_add_effects'] = list()
        if operator_details.op.at_end_add_effects:
            number_pred = len(operator_details.op.at_end_add_effects)
            for i in range(number_pred):
                self.effects['at_end_add_effects'].append(list())
                predicate_name = operator_details.op.at_end_add_effects[i].name
                self.effects['at_end_add_effects'][i].append(predicate_name)
                number_param = len(operator_details.op.at_end_add_effects[i].typed_parameters)
                for j in range(number_param):
                    variable = operator_details.op.at_end_add_effects[i].typed_parameters[j].key
                    self.effects['at_end_add_effects'][i].append(variable)

        # at_end_del_effects
        self.effects['at_end_del_effects'] = list()
        if operator_details.op.at_end_del_effects:
            number_pred = len(operator_details.op.at_end_del_effects)
            for i in range(number_pred):
                self.effects['at_end_del_effects'].append(list())
                predicate_name = operator_details.op.at_end_del_effects[i].name
                self.effects['at_end_del_effects'][i].append(predicate_name)
                number_param = len(operator_details.op.at_end_del_effects[i].typed_parameters)
                for j in range(number_param):
                    variable = operator_details.op.at_end_del_effects[i].typed_parameters[j].key
                    self.effects['at_end_del_effects'][i].append(variable)

        # at_start_assign_effects
        self.effects['at_start_assign_effects'] = list()
        if operator_details.op.at_start_assign_effects:
            number_pred = len(operator_details.op.at_start_assign_effects)
            for i in range(number_pred):
                self.effects['at_start_assign_effects'].append(list())
                predicate_name = operator_details.op.at_start_assign_effects[i].name
                self.effects['at_start_assign_effects'][i].append(predicate_name)
                number_param = len(operator_details.op.at_start_assign_effects[i].typed_parameters)
                for j in range(number_param):
                    variable = operator_details.op.at_start_assign_effects[i].typed_parameters[j].key
                    self.effects['at_start_assign_effects'][i].append(variable)

        # at_end_assign_effects
        self.effects['at_end_assign_effects'] = list()
        if operator_details.op.at_end_assign_effects:
            number_pred = len(operator_details.op.at_end_assign_effects)
            for i in range(number_pred):
                self.effects['at_end_assign_effects'][i].append(list())
                predicate_name = operator_details.op.at_end_assign_effects[i].name
                self.effects['at_end_assign_effects'][i].append(predicate_name)
                number_param = len(operator_details.op.at_end_assign_effects[i].typed_parameters)
                for j in range(number_param):
                    variable = operator_details.op.at_end_assign_effects[i].typed_parameters[j].key
                    self.effects['at_end_assign_effects'][i].append(variable)

        # probabilistic_effects
        self.effects['probabilistic_effects'] = list()
        if operator_details.op.probabilistic_effects:
            self.effects['probabilistic_effects'].append[list()]
            number_pred = len(operator_details.op.probabilistic_effects)
            for i in range(number_pred):
                predicate_name = operator_details.op.probabilistic_effects[i].name
                self.effects['probabilistic_effects'][i].append(predicate_name)
                number_param = len(operator_details.op.probabilistic_effects[i].typed_parameters)
                for j in range(number_param):
                    variable = operator_details.op.probabilistic_effects[i].typed_parameters[j].key
                    self.effects['probabilistic_effects'][i].append(variable)

        # at_start_simple_condition
        self.conditions['at_start_simple_condition'] = list()
        if operator_details.op.at_start_simple_condition:
            number_pred = len(operator_details.op.at_start_simple_condition)
            for i in range(number_pred):
                self.conditions['at_start_simple_condition'].append(list())
                predicate_name = operator_details.op.at_start_simple_condition[i].name
                self.conditions['at_start_simple_condition'][i].append(predicate_name)
                number_param = len(operator_details.op.at_start_simple_condition[i].typed_parameters)
                for j in range(number_param):
                    variable = operator_details.op.at_start_simple_condition[i].typed_parameters[j].key
                    self.conditions['at_start_simple_condition'][i].append(variable)

        # over_all_simple_condition
        self.conditions['over_all_simple_condition'] = list()
        if operator_details.op.over_all_simple_condition:
            number_pred = len(operator_details.op.over_all_simple_condition)
            for i in range(number_pred):
                self.conditions['over_all_simple_condition'].append(list())
                predicate_name = operator_details.op.over_all_simple_condition[i].name
                self.conditions['over_all_simple_condition'][i].append(predicate_name)
                number_param = len(operator_details.op.over_all_simple_condition[i].typed_parameters)
                for j in range(number_param):
                    variable = operator_details.op.over_all_simple_condition[i].typed_parameters[j].key
                    self.conditions['over_all_simple_condition'][i].append(variable)

        # at_end_simple_condition
        self.conditions['at_end_simple_condition'] = list()
        if operator_details.op.at_end_simple_condition:
            number_pred = len(operator_details.op.at_end_simple_condition)
            for i in range(number_pred):
                self.conditions['at_end_simple_condition'].append(list())
                predicate_name = operator_details.op.at_end_simple_condition[i].name
                self.conditions['at_end_simple_condition'][i].append(predicate_name)
                number_param = len(operator_details.op.at_end_simple_condition[i].typed_parameters)
                for j in range(number_param):
                    variable = operator_details.op.at_end_simple_condition[i].typed_parameters[j].key
                    self.conditions['at_end_simple_condition'][i].append(variable)

        # at_start_neg_condition
        self.conditions['at_start_neg_condition'] = list()
        if operator_details.op.at_start_neg_condition:
            number_pred = len(operator_details.op.at_start_neg_condition)
            for i in range(number_pred):
                self.conditions['at_start_neg_condition'].append(list())
                predicate_name = operator_details.op.at_start_neg_condition[i].name
                self.conditions['at_start_neg_condition'][i].append(predicate_name)
                number_param = len(operator_details.op.at_start_neg_condition[i].typed_parameters)
                for j in range(number_param):
                    variable = operator_details.op.at_start_neg_condition[i].typed_parameters[j].key
                    self.conditions['at_start_neg_condition'][i].append(variable)

        # over_all_neg_condition
        self.conditions['over_all_neg_condition'] = list()
        if operator_details.op.over_all_neg_condition:
            number_pred = len(operator_details.op.over_all_neg_condition)
            for i in range(number_pred):
                self.conditions['over_all_neg_condition'].append(list())
                predicate_name = operator_details.op.over_all_neg_condition[i].name
                self.conditions['over_all_neg_condition'][i].append(predicate_name)
                number_param = len(operator_details.op.over_all_neg_condition[i].typed_parameters)
                for j in range(number_param):
                    variable = operator_details.op.over_all_neg_condition[i].typed_parameters[j].key
                    self.conditions['over_all_neg_condition'][i].append(variable)

        # at_end_neg_condition
        self.conditions['at_end_neg_condition'] = list()
        if operator_details.op.at_end_neg_condition:
            number_pred = len(operator_details.op.at_end_neg_condition)
            for i in range(number_pred):
                self.conditions['at_end_neg_condition'].append(list())
                predicate_name = operator_details.op.at_end_neg_condition[i].name
                self.conditions['at_end_neg_condition'][i].append(predicate_name)
                number_param = len(operator_details.op.at_end_neg_condition[i].typed_parameters)
                for j in range(number_param):
                    variable = operator_details.op.at_end_neg_condition[i].typed_parameters[j].key
                    self.conditions['at_end_neg_condition'][i].append(variable)

        # at_start_comparison
        self.comparison['at_start_comparison'] = list()
        if operator_details.op.at_start_comparison:
            number_pred = len(operator_details.op.at_start_comparison)
            for i in range(number_pred):
                self.comparison['at_start_comparison'].append(list())
                predicate_name = operator_details.op.at_start_comparison[i].name
                self.comparison['at_start_comparison'][i].append(predicate_name)
                number_param = len(operator_details.op.at_start_comparison[i].typed_parameters)
                for j in range(number_param):
                    variable = operator_details.op.at_start_comparison[i].typed_parameters[j].key
                    self.comparison['at_start_comparison'][i].append(variable)

        # at_end_comparison
        self.comparison['at_end_comparison'] = list()
        if operator_details.op.at_end_comparison:
            number_pred = len(operator_details.op.at_end_comparison)
            for i in range(number_pred):
                self.comparison['at_end_comparison'].append(list())
                predicate_name = operator_details.op.at_end_comparison[i].name
                self.comparison['at_end_comparison'][i].append(predicate_name)
                number_param = len(operator_details.op.at_end_comparison[i].typed_parameters)
                for j in range(number_param):
                    variable = operator_details.op.at_end_comparison[i].typed_parameters[j].key
                    self.comparison['at_end_comparison'][i].append(variable)

        # over_all_comparison
        self.comparison['over_all_comparison'] = list()
        if operator_details.op.over_all_comparison:
            number_pred = len(operator_details.op.over_all_comparison)
            for i in range(number_pred):
                self.comparison['over_all_comparison'].append(list())
                predicate_name = operator_details.op.over_all_comparison[i].name
                self.comparison['over_all_comparison'][i].append(predicate_name)
                number_param = len(operator_details.op.over_all_comparison[i].typed_parameters)
                for j in range(number_param):
                    variable = operator_details.op.over_all_comparison[i].typed_parameters[j].key
                    self.comparison['over_all_comparison'][i].append(variable)

        Action.actionsList.append(self)
    

    def printAction(self):
        print('>>>>>>>>>>>>>>>>>>>>>>>>')
        print('Action name: ' + self.name)
        print('Action duration: ' + str(self.duration))
        print('Conditions: ' + str(self.conditions))
        print('Effects: ' + str(self.effects))
        print('Comparison: ' + str(self.comparison))
        print('<<<<<<<<<<<<<<<<<<<<<<<<')

    @classmethod
    def getAction(cls, action_name):
        for item in Action.actionsList:
            if item.name == action_name:
                return item
        return None

    @classmethod
    def getActionNamesList(cls):
        names_list = list()
        for item in Action.actionsList:
            names_list.append(item.name)
        return names_list



class GroundedAction:

    actionsList = list()

    def __init__(self, action, list_objects):
        # Action duration is missing
        variables = action.parameters
        var_obj = dict(zip(variables, list_objects))
        self.name = str( action.name + '#' + '#'.join(str(v) for v in list_objects))

        ### This block of code defines the effects as they were in Action, but replaces the variables
        ### with objects, e.g. ['robot_at', 'r', 'dest'] -> ['robot_at', 'mbot', 'wp1']
        self.effects = dict()
        # For each key and value of action effects
        for k, v in action.effects.items():
            # Initialise value of grounded actions effects as list (e.g. k = 'at_start_add_effects')
            self.effects[k] = list()
            # If the key has a value, meaning it has at_start_add_effects, for example
            if v:
                i = 0
                # e.g., for each effect, e.g. ['robot_at', 'r', 'dest']
                for value in v:
                    # append a list to the value of effects['at_start_add_effects']
                    self.effects[k].append(list())
                    # For each word in effect, e.g. 'robot_at', 'r' and 'dest'
                    for item in value:
                        # If word is in var_obj.keys(), meaning if it's a variable
                        if item in list(var_obj.keys()):
                            # Then append the object associated with that variable, e.g. 'r'->'mbot'
                            self.effects[k][i].append(var_obj[item])
                        else:
                            # If it's not a variable, then just append it, e.g. 'robot_at'
                            self.effects[k][i].append(item)
                    i = i + 1
            # If it has no effects (empty), then just define it as it is
            else:
                self.effects[k] = v
        
        ### This does the same as the above but for conditions
        self.conditions = dict()
        for k, v in action.conditions.items():
            self.conditions[k] = list()
            if v:
                i = 0
                for value in v:
                    self.conditions[k].append(list())
                    for item in value:
                        if item in list(var_obj.keys()):
                            self.conditions[k][i].append(var_obj[item])
                        else:
                            self.conditions[k][i].append(item)
                    i = i + 1
            else:
                self.conditions[k] = v

        ### Same as the, but for comparison
        self.comparison = dict()
        for k, v in action.comparison.items():
            self.comparison[k] = list()
            if v:
                for value in v:
                    self.comparison[k].append(list())
                    for item in value:
                        if item in list(var_obj.keys()):
                            self.comparison[k][i].append(var_obj[item])
                        else:
                            self.comparison[k][i].append(item)
                    i = i + 1
            else:
                self.comparison[k] = v
        
        GroundedAction.actionsList.append(self)


    def getPredicates(self):
        predicates_set = set()
        for k, v in self.conditions.items():
            for predicate in v:
                predicates_set.add('#'.join(predicate))
        for k, v in self.effects.items():
            for predicate in v:
                predicates_set.add('#'.join(predicate))
        for k, v in self.comparison.items():
            for predicate in v:
                predicates_set.add('#'.join(predicate))
        
        return predicates_set


    @classmethod
    def getAllPredicates(cls):
        predicates_set = set()
        for action in GroundedAction.actionsList:
            for predicate in action.getPredicates():
                predicates_set.add(predicate)

        return predicates_set


    def getString(self):
        string = str()
        string = string + '>>>>>>>>>>>>>>>>>>>>>>>>\n' + 'Grounded Action name: ' + self.name + '\n' + \
                 'Conditions: ' + str(self.conditions) + '\n' + 'Effects: ' + str(self.effects) + '\n' + \
                 '<<<<<<<<<<<<<<<<<<<<<<<<'
        return string
        # print('>>>>>>>>>>>>>>>>>>>>>>>>')
        # print('Grounded Action name: ' + self.name)
        # # print('Action duration: ' + str(self.duration))
        # print('Conditions: ' + str(self.conditions))
        # print('Effects: ' + str(self.effects))
        # # print('Comparison: ' + str(self.comparison))
        # print('<<<<<<<<<<<<<<<<<<<<<<<<')


    @classmethod
    def getGroundedAction(cls, action_name):
        for item in GroundedAction.actionsList:
            if item.name == action_name:
                return item
        return None


    @classmethod
    def getGroundedActionsList(cls):
        list_actions = list()
        for grounded_action in GroundedAction.actionsList:
            list_actions.append(grounded_action.name)
        return list_actions



class ActionStart:

    actionsList = list()

    def __init__(self, grounded_action):
        name_act = grounded_action.name.split('#')
        name_act[0] = name_act[0] + '_start'
        self.name = '#'.join(name_act)
        # Name format: goto_waypoint#robot0#wp0#machine

        self.conditions = dict()
        for k, v in grounded_action.conditions.items():
            if k[:8] == 'at_start':
                self.conditions[k[9:]] = grounded_action.conditions[k]
        
        self.effects = dict()
        for k, v in grounded_action.effects.items():
            if k[:8] == 'at_start':
                self.effects[k[9:]] = grounded_action.effects[k]
        
        ActionStart.actionsList.append(self)


    def getPredicates(self):
        predicates_set = set()
        for k, v in self.conditions.items():
            for predicate in v:
                predicates_set.add('#'.join(predicate))
        for k, v in self.effects.items():
            for predicate in v:
                predicates_set.add('#'.join(predicate))
        for k, v in self.comparison.items():
            for predicate in v:
                predicates_set.add('#'.join(predicate))
        
        return predicates_set


    def getConditionPredicates(self):
        predicates_set = set()
        for k, v in self.conditions.items():
            for predicate in v:
                predicates_set.add('#'.join(predicate))
        
        return predicates_set


    def getEffectsPredicates(self):
        predicates_set = set()
        for k, v in self.effects.items():
            for predicate in v:
                predicates_set.add('#'.join(predicate))
        
        return predicates_set


    def printAction(self):
        print('>>>>>>>>>>>>>>>>>>>>>>>>')
        print('ActionStart name: ' + self.name)
        print('Conditions: ' + str(self.conditions))
        print('Effects: ' + str(self.effects))
        print('<<<<<<<<<<<<<<<<<<<<<<<<')


    @classmethod
    def getActionStart(cls, action_name):
        for item in ActionStart.actionsList:
            if item.name == action_name:
                return item
        return None



class ActionEnd:

    actionsList = list()


    def __init__(self, grounded_action):
        name_act = grounded_action.name.split('#')
        name_act[0] = name_act[0] + '_end'
        self.grounded_action = grounded_action
        self.name = '#'.join(name_act)

        self.conditions = dict()
        for k, v in grounded_action.conditions.items():
            if k[:6] == 'at_end':
                self.conditions[k[7:]] = grounded_action.conditions[k]

        self.over_all_conditions = dict()
        for k, v in grounded_action.conditions.items():
            if k[:8] == 'over_all':
                self.over_all_conditions[k[9:]] = grounded_action.conditions[k]

        ### I don't think over all effects make sense
        # self.over_all_effects = dict()
        # for k, v in grounded_action.effects.items():
        #     if k[:8] == 'over_all':
        #         self.over_all_effects[k[9:]] = grounded_action.effects[k]

        self.effects = dict()
        for k, v in grounded_action.effects.items():
            if k[:6] == 'at_end':
                self.effects[k[7:]] = grounded_action.effects[k]

        ActionEnd.actionsList.append(self)


    def getPredicates(self):
        predicates_set = set()
        for k, v in self.conditions.items():
            for predicate in v:
                predicates_set.add('#'.join(predicate))
        for k, v in self.over_all_conditions.items():
            for predicate in v:
                predicates_set.add('#'.join(predicate))
        for k, v in self.effects.items():
            for predicate in v:
                predicates_set.add('#'.join(predicate))
        for k, v in self.over_all_effects.items():
            for predicate in v:
                predicates_set.add('#'.join(predicate))
        
        return predicates_set


    def printAction(self):
        print('>>>>>>>>>>>>>>>>>>>>>>>>')
        print('ActionEnd name: ' + self.name)
        print('Conditions: ' + str(self.conditions))
        print('Effects: ' + str(self.effects))
        print('Over all conditions: ' + str(self.over_all_conditions))
        print('Over all effects: ' + str(self.over_all_effects))
        print('<<<<<<<<<<<<<<<<<<<<<<<<')


    def getConditionPredicates(self):
        predicates_set = set()
        for k, v in self.conditions.items():
            for predicate in v:
                predicates_set.add('#'.join(predicate))
        
        return predicates_set


    def getOverAllPredicates(self):
        predicates_set = set()
        for k, v in self.over_all_conditions.items():
            for predicate in v:
                predicates_set.add('#'.join(predicate))
        
        return predicates_set


    def getEffectsPredicates(self):
        predicates_set = set()
        for k, v in self.effects.items():
            for predicate in v:
                predicates_set.add('#'.join(predicate))
        
        return predicates_set


    @classmethod
    def getActionEnd(cls, action_name):
        for item in ActionEnd.actionsList:
            if item.name == action_name:
                return item
        return None


    def getActionStart(self):
        name_act = self.grounded_action.name.split('#')
        name_act[0] = name_act[0] + '_start'
        name = '#'.join(name_act)
        return ActionStart.getActionStart(name)



def isPredicate(node_name):
    return len(node_name.split('%')) > 1


######### SIMPLIFY NAMES #########
def removeStartEndFromName(name):
    ''' Removes time from action name, works with or without parameters in name '''
    name1 = name.split('#')
    name2 = name1[0].split('_')
    del name2[-1]
    name3 = '_'.join(name2)
    params = '#'.join(name1[1:])
    if params:
        return name3+'#'+params
    return name3


def removeStartEndParamsFromName(name):
    name1 = removeStartEndFromName(name)
    return name1.split('#')[0]


#########    PRINTS     #########
def printNodesList(nodes_list, msg):
    names_list = list()
    for node in nodes_list:
        names_list.append(node.name)
    print(msg + str(names_list))


def printPlan(plan, action_times):
    print('>>> PRINTING PLAN')
    for i in action_times:
        print('> Time: ' + str(i))
        for action in ActionStart.getActionsWithTime(i):
            print(action.name)
        for action in ActionEnd.getActionsWithTime(i):
            print(action.name)
    print('---------------------------------')


######### WRITE TO FILE #########
def writePredicatesToFile():
    file = open('predicate_layers.txt', 'w')
    file.write('PREDICATE LAYERS:\n')
    for node_name in all_nodes_:
        if isPredicate(node_name):
            file.write(node_name+'\n')
    # for layer_num in range(plan_length+1):
    #     file.write('>> Layer: ' + str(layer_num) + '\n')
    #     for predicate in all_predicates:
    #         file.write(predicate + '%' + str(layer_num) + '\n')
    #     file.write('----------------------------\n')
    file.close()


def writePredicateParentsToFile(predicates_par_child):
    file = open('nodes_parents.txt', 'w')
    file.write('NODES AND PARENTS:\n')
    # For loop to print nodes in order
    for i in range(0, len(predicates_par_child.keys())):
        for predicate in predicates_par_child.keys():
            node_index = predicate.split('%')[1]
            if node_index == str(i):
                file.write('>>> Node: ' + predicate + '\n')
                for par in predicates_par_child[predicate]['parents']:
                    file.write(par + '\n')
                file.write('----------------------------\n')
    file.close()


def getPredicateParentsAsString(predicates_par_child):
    string = 'NODES AND PARENTS:\n'
    # For loop to print nodes in order
    for i in range(0, len(predicates_par_child.keys())):
        for predicate in predicates_par_child.keys():
            node_index = predicate.split('%')[1]
            if node_index == str(i):
                string = string + '>>> Node: ' + predicate + '\n'
                for par in predicates_par_child[predicate]['parents']:
                    string = string + par + '\n'
                string = string + '----------------------------\n'
    return string


def writeActionsToFile(actions_par_child):
    file = open('actions_par_child.txt', 'w')
    file.write('ACTIONS WITH CHILDREN AND PARENTS:\n')
    # This bigger for cycle is to print the actions in order
    for i in range(1, len(actions_par_child.keys())+1):
        for action_name in actions_par_child.keys():
            action_index = action_name.split('$')[1]
            if action_index == str(i):
                file.write('>>> ACTION: ' + action_name + '\n')
                file.write('> Parents:\n')
                for par in actions_par_child[action_name]['parents']:
                    file.write(par + '\n')
                file.write('> Children:\n')
                for child in actions_par_child[action_name]['children']:
                    file.write(child + '\n')
                file.write('----------------------------\n')
    file.close()


def writeNodesAndCPDsToFile():
    file = open('nodes_cpds.txt', 'w')
    file.write('NODES AND CPDs:\n')
    for node in all_nodes_:
        file.write('>>> Node: ' + node + '\n')
        file.write(str(cpds_map_[node]) + '\n')
    file.close()


######### NODE CHECKING #########
def checkNodesWithoutParents(parents):
    without_parents = False
    for node in parents.keys():
        first_layer = False
        if len(node.name.split('%')) > 1:
            if node.name.split('%')[1] == '0':
                first_layer = True
        if not first_layer and len(parents[node]) == 0:
            without_parents = True
            print('!!!! Node without parents !!!!')
            print('>> Node without parents: ' + node.name)
    if not without_parents:
        print('  No nodes without parents')


def checkNodesWithoutChildren(children, plan):
    without_children = False
    for node in children.keys():
        last_layer = False
        if len(node.name.split('%')) > 1:
            size = len(plan)
            if node.name.split('%')[1] == str(size):
                last_layer = True
        if not last_layer and len(children[node]) == 0:
            without_children = True
            print('!!!! Node without children !!!!')
            print('>> Node without children: ' + node.name)
    if not without_children:
        print('  No nodes without children')


def checkForRepeatedNodes():
    copy_set = set(all_nodes_)
    if len(all_nodes_) == len(copy_set):
        print('  No repeated nodes')
    else:
        print('!!! REPEATED NODES !!!')


######### CREATE ACTIONS #########
def createActions(operators):
    ''' Creates action instances for all actions in domain file '''
    for i in range(len(operators)):
        action_name = operators[i].name
        Action(action_name)


def createGroundedActions(original_plan):
    for action_name in original_plan:
        name = removeStartEndParamsFromName(action_name)
        if not Action.getAction(name):
            action = Action(name)
        else:
            action = Action.getAction(name)
        GroundedAction(action, action_name.split('#')[1:])


def cyclesRecurse(children, nodes_list, node):
    if len(children[node]) == 0:
        del nodes_list[-1]
        return True
    for child in children[node]:
        if child in nodes_list:
            printNodesList(nodes_list)
            print('>> Repeated node: ' + child.name)
            return False
        nodes_list.append(child)
        if not cyclesRecurse(children, nodes_list, child):
            return False
    del nodes_list[-1]
    return True


def checkCycles(children):
    for node in all_nodes_:
        # children receives nodes and not strings, got to get node with this string as name
        nodes_list = list()
        nodes_list.append(node)
        for child in children[node]:
            if child in nodes_list:
                return False
            nodes_list.append(child)
            if not cyclesRecurse(children, nodes_list, child):
                return False
    return True


######### ADD PREDICATES #########
def addPredicate(predicate, predicates_par_child, layer_number):
    global all_nodes_
    prev_pred_name = predicate + '%' + str(layer_number-1)
    pred_name = predicate + '%' + str(layer_number)
    all_nodes_.append(pred_name)
    predicates_par_child[pred_name] = dict()
    predicates_par_child[pred_name]['parents'] = set()
    predicates_par_child[pred_name]['children'] = set()
    predicates_par_child[pred_name]['parents'].add(prev_pred_name)
    predicates_par_child[prev_pred_name]['children'].add(pred_name)


def addPredicateLayer(predicates_set, layer_number, predicates_par_child):
    for predicate in predicates_set:
        predicate_name = predicate + '%' + str(layer_number)
        if not predicate_name in all_nodes_:
            addPredicate(predicate, predicates_par_child, layer_number)


def addFirstLayerPredicates(predicates_set, predicates_par_child):
    global all_nodes_
    for predicate in predicates_set:
        pred_name = predicate + '%0'
        all_nodes_.append(pred_name)
        predicates_par_child[pred_name] = dict()
        predicates_par_child[pred_name]['parents'] = set()
        predicates_par_child[pred_name]['children'] = set()



######### CONNECT PREDICATES #########
def connectActionToConditionPredicates(action, action_name, predicates_par_child, actions_par_child, layer_number):
    for predicate in action.getConditionPredicates():
        pred_name = predicate + '%' + str(layer_number-1)
        # node_predicate = get_node(all_nodes_, predicate + '%' + str(layer_number-1))
        # model.add_edge(node_predicate, action_node)
        predicates_par_child[pred_name]['children'].add(action_name)
        actions_par_child[action_name]['parents'].add(pred_name)


def connectActionToEffectsPredicates(action, action_name, predicates_par_child, actions_par_child, layer_number):
    global all_nodes_
    
    for predicate in action.getEffectsPredicates():
        # Add predicate to model
        pred_name = predicate + '%' + str(layer_number)
        addPredicate(predicate, predicates_par_child, layer_number)
        # Connect predicate to predicate in previous layer
        prev_pred_name = predicate + '%' + str(layer_number-1)
        predicates_par_child[pred_name]['parents'].add(action_name)
        predicates_par_child[pred_name]['parents'].add(prev_pred_name)
        predicates_par_child[prev_pred_name]['children'].add(pred_name)
        # Connect predicate to action
        actions_par_child[action_name]['children'].add(pred_name)
        


def connectActionEndToActionStart(action, action_name, end_index, actions_par_child):
    action_start = action.getActionStart()
    # Search for the layer where actionStart is
    for j in range(1, end_index+1):
        action_start_name = action_start.name + '$' + str(j)
        if action_start_name in all_nodes_:
            # Connecting ActionEnd to ActionStart
            actions_par_child[action_name]['parents'].add(action_start_name)
            actions_par_child[action_start_name]['children'].add(action_name)
            return j
    return 0


def connectActionToOverAllPredicates(action, action_name, start_index, end_index, actions_par_child, predicates_par_child):
    for predicate in action.getOverAllPredicates():
        for j in range(start_index, end_index):
            pred_name = predicate + '%' + str(j)
            actions_par_child[action_name]['parents'].add(pred_name)
            predicates_par_child[pred_name]['children'].add(action_name)


def addActionEdges(action, action_name, predicates_par_child, actions_par_child, layer_number):
    actions_par_child[action_name] = dict()
    actions_par_child[action_name]['parents'] = set()
    actions_par_child[action_name]['children'] = set()

    connectActionToConditionPredicates(action, action_name, predicates_par_child, actions_par_child, layer_number)
    connectActionToEffectsPredicates(action, action_name, predicates_par_child, actions_par_child, layer_number)

    if isinstance(action, ActionEnd):
        end_index = layer_number
        start_index = connectActionEndToActionStart(action, action_name, end_index, actions_par_child)
        # If action has no action_start then do not connect to over all predicates
        connectActionToOverAllPredicates(action, action_name, start_index, end_index, actions_par_child, predicates_par_child)


######### REMOVE NODE #########
def removeNode(node, actions_par_child, predicates_par_child, is_predicate):    
    global all_nodes_
    if is_predicate:
        predicates_par_child.pop(node)
    else:
        actions_par_child.pop(node)

    all_nodes_.remove(node)
    # cpds_map_.pop(node)

    for pred in predicates_par_child.keys():
        if node in predicates_par_child[pred]['parents']:
            predicates_par_child[pred]['parents'].remove(node)
        if node in predicates_par_child[pred]['children']:
            predicates_par_child[pred]['children'].remove(node)

    for action in actions_par_child.keys():
            if node in actions_par_child[action]['parents']:
                actions_par_child[action]['parents'].remove(node)
            if node in actions_par_child[action]['children']:
                actions_par_child[action]['children'].remove(node)


######### PRUNE NETWORK #########
def pruneNetwork(actions_par_child, predicates_par_child):
    size = len(actions_par_child)
    goal_with_time = set()
    for item in goal_:
        goal_with_time.add(item + '%' + str(size))

    for node in reversed(all_nodes_):
        is_predicate = isPredicate(node)

        # print('>>> Node: ' + node)
        # if isPredicate:
        #     print('> Children: ' + str(predicates_par_child[node]['children']))
        #     print('> Parents: ' + str(predicates_par_child[node]['parents']))
        # else:
        #     print('> Children: ' + str(actions_par_child[node]['children']))
        #     print('> Parents: ' + str(actions_par_child[node]['parents']))

        if is_predicate:
            ##### Rule 1 #####
            # If predicate does not have children and is not the goal_, then remove it
            if not node in goal_with_time:
                if not predicates_par_child[node]['children']:
                    removeNode(node, actions_par_child, predicates_par_child, is_predicate)
                    # Skip to next node in for loop
                    continue

            has_action_as_parent = False
            for parent in predicates_par_child[node]['parents']:
                if not isPredicate(parent):
                    has_action_as_parent = True
                    parent_action = parent
                    break
            
            ##### Rule 3 #####
            ### If predicate is children of action, then remove
            ### edges from other parents and replace CPD
            if has_action_as_parent:
                removed_parents = set()
                for par in predicates_par_child[node]['parents']:
                    # if parent is predicate, then remove edges
                    if isPredicate(par):
                        removed_parents.add(par)
                # Had to remove it here and not inside the previous cycle because
                # I can't change the size of a list while iterating over it
                for par in removed_parents:
                    predicates_par_child[par]['children'].remove(node)
                    predicates_par_child[node]['parents'].remove(par)
                
                action_name_no_time = removeStartEndFromName(parent_action).split('$')[0]
                predicate_no_parameters = node.split('#')[0]
                effects_success = action_probabilities_map_[action_name_no_time][1][predicate_no_parameters]
                action_cpd = cpds_map_[parent_action]
                cpds_map_[node] = ConditionalProbabilityTable(
                                        [['T', 'T', effects_success],
                                        ['T', 'F', 1-effects_success],
                                        ['F', 'T', 0],
                                        ['F', 'F', 1]], [action_cpd])


            ##### Rule 2 #####
            ### If predicate is precondition of action_name then change its CPD to true
            # If node is parent of an action and has as parent a predicate, then change its CPD to true
            # We check if it has a predicate as parent because the predicates in the first layer don't
            # and the CPD would be different. We don't care about the predicates in the first layer
            # because the fact they are the initial state already defines their CPD
            if not node.split('%')[1] == str(0):

                has_action_as_child = False
                for child in predicates_par_child[node]['children']:
                    if not isPredicate(child):
                        has_action_as_child = True
                        child_action = child
                        break

                if has_action_as_child:

                    # Get all combinations of 'F' and 'T' has a list of len(actions_par_child[node]['parents'])+1 items
                    # I add 1 because the last entry represents the node itself
                    cpd_lines = list(itertools.product(['F', 'T'], repeat=len(predicates_par_child[node]['parents'])+1))

                    cpd_elements = list()

                    parents_cpd_list = list()
                    for predicate in predicates_par_child[node]['parents']:
                        parents_cpd_list.append(cpds_map_[predicate])
                    parents_cpd_list.append(cpds_map_[parent_action])

                    for line in cpd_lines:
                        lst = list(line)
                        if all([elem == 'T' for elem in lst]):
                            lst.append(1)
                        elif all([elem == 'T' for elem in lst[:-1]]):
                            lst.append(0)
                        elif lst[-1] == 'T':
                            lst.append(0)
                        else:
                            lst.append(1)
                        cpd_elements.append(lst)      
                    
                    cpd = ConditionalProbabilityTable(cpd_elements, parents_cpd_list)
                    cpds_map_[node] = cpd
                


######### BUILD NETWORK IN MODEL #########
def buildNetworkInModel(model, actions_par_child, predicates_par_child):
    nodes_dict = dict()
    for node_name in all_nodes_:        
        cpd = cpds_map_[node_name]
        node = Node(cpd, name=node_name)
        nodes_dict[node_name] = node
        model.add_node(node)

        if isPredicate(node_name):
            parents_dict = predicates_par_child
        else:
            parents_dict = actions_par_child
      
        for parent in parents_dict[node_name]['parents']:
            parent_node = nodes_dict[parent]
            model.add_edge(parent_node, node)


######### BUILD CPDs #########
# # TODO: This needs to account for true and false predicates
def buildCPDs(actions_par_child, predicates_par_child):

    for node in all_nodes_:

        if isPredicate(node):

            node_without_time = node.split('%')[0]

            # If node is in first layer
            if int(node.split('%')[1]) == 0:
                # If it is in initial state then node is true
                if node.split('%')[0] in initial_state_:
                    cpd = DiscreteDistribution({'T': 1, 'F': 0})
                    cpds_map_[node] = cpd
                    continue
                # Else it is false
                cpd = DiscreteDistribution({'T': 0, 'F': 1})
                cpds_map_[node] = cpd
                continue

            # If predicate only has one parent, which can only be
            # the same predicate in the previous layer
            if len(predicates_par_child[node]['parents']) == 1:
                spont_false_true = pred_probabilities_map_[node_without_time][0]
                spont_true_false = pred_probabilities_map_[node_without_time][1]
                parent = list(predicates_par_child[node]['parents'])[0]
                parent_cpd = cpds_map_[parent]
                cpd = ConditionalProbabilityTable(
                        [['T', 'T', 1-spont_true_false],
                        ['T', 'F', spont_true_false],
                        ['F', 'T', spont_false_true],
                        ['F', 'F', 1-spont_false_true]], [parent_cpd])
                cpds_map_[node] = cpd
                continue

            # If predicate has more than one parent, then it has two:
            # the predicate in the previous layer and an action
            else:
                for parent in predicates_par_child[node]['parents']:
                    if isPredicate(parent):
                        parent_predicate = parent
                    else:
                        parent_action = parent

                try:
                    parent_predicate
                except NameError:
                    rospy.logerr('Predicate has no predicate as parent')
                try:
                    parent_action
                except NameError:
                    rospy.logerr('Predicate has no action as parent')

                parent_predicate_cpd = cpds_map_[parent_predicate]
                parent_action_cpd = cpds_map_[parent_action]

                spont_false_true = pred_probabilities_map_[node_without_time][0]
                spont_true_false = pred_probabilities_map_[node_without_time][1]

                action_name_without_time = removeStartEndFromName(parent_action).split('$')[0]
                predicate_without_parameters = node.split('#')[0]
                effects_success = action_probabilities_map_[action_name_without_time][1][predicate_without_parameters]

                cpd = ConditionalProbabilityTable(
                            [['F', 'F', 'T', spont_false_true],
                            ['F', 'F', 'F', 1-spont_false_true],
                            ['F', 'T', 'T', 1-spont_true_false],
                            ['F', 'T', 'F', spont_true_false],
                            ['T', 'F', 'T', effects_success],
                            ['T', 'F', 'F', 1-effects_success],
                            ['T', 'T', 'T', effects_success],
                            ['T', 'T', 'F', 1-effects_success]], [parent_action_cpd, parent_predicate_cpd])
                cpds_map_[node] = cpd
        

        # If node is an action
        else:
            parents_predicates = set()
            for parent in actions_par_child[node]['parents']:
                if isPredicate(parent):
                    parents_predicates.add(parent)
                # Each action can only have 1 action as parent, at most (in case is at_end action)
                else:
                    parent_action = parent

            is_at_end_action = True
            # If parent_action is not defined then action has no other
            # action as parent hence it's an at_start action
            try:
                parent_action
            except NameError:
                is_at_end_action = False

            if is_at_end_action:
                parents_cpd_list = list()
                for predicate in parents_predicates:
                    parents_cpd_list.append(cpds_map_[predicate])
                parents_cpd_list.append(cpds_map_[parent_action])

                # Added 1 to account for the node itself, if it's true or false
                cpd_lines = list(itertools.product(['F', 'T'], repeat=len(parents_cpd_list)+1))

                action_cpd_elements = list()
                for i in range(len(cpd_lines)):
                    lst = list(cpd_lines[i])
                    if all([elem == 'T' for elem in lst]):
                        lst.append(1)
                    elif all([elem == 'T' for elem in lst[:-1]]):
                        lst.append(0)
                    elif lst[-1] == 'T':
                        lst.append(0)
                    else:
                        lst.append(1)
                    action_cpd_elements.append(lst)

                cpd = ConditionalProbabilityTable(action_cpd_elements, parents_cpd_list)

                cpds_map_[node] = cpd

            else:
                parents_cpd_list = list()
                for predicate in parents_predicates:
                    parents_cpd_list.append(cpds_map_[predicate])

                # Added 1 to account for the node itself, if it's true or false
                cpd_lines = list(itertools.product(['F', 'T'], repeat=len(parents_cpd_list)+1))

                action_name_without_time = removeStartEndFromName(node).split('$')[0]
                success_prob = action_probabilities_map_[action_name_without_time][0]

                action_cpd_elements = list()
                for line in cpd_lines:
                    lst = list(line)
                    # If all the preconditions are true and the action is successful
                    if all([elem == 'T' for elem in lst]):
                        lst.append(success_prob)
                    # If all the preconditions are true and the action fails
                    elif all([elem == 'T' for elem in lst[:-1]]):
                        lst.append(1-success_prob)
                    # If not all the preconditions are true and the action is successful
                    elif lst[-1] == 'T':
                        lst.append(0)
                    # If not all the preconditions are true and the action fails
                    else:
                        lst.append(1)
                    action_cpd_elements.append(lst)

                cpd = ConditionalProbabilityTable(action_cpd_elements, parents_cpd_list)

                cpds_map_[node] = cpd


######### PARSE PLAN #########
def convertPlanToActionStart_End(original_plan):
    plan = list()
    for action_name in original_plan:
        name = removeStartEndFromName(action_name)
        grounded_action = GroundedAction.getGroundedAction(name)
        if action_name.split('#')[0][-5:] == 'start':
            action_start = ActionStart(grounded_action)
            plan.append(action_start)
        elif action_name.split('#')[0][-3:] == 'end':
            action_end = ActionEnd(grounded_action)
            plan.append(action_end)
    return plan


######### GET STUFF #########
def getProbabilities():
    global pred_probabilities_map_
    global action_probabilities_map_
    
    file = open('/home/tomas/ros_ws/src/ROSPlan/src/rosplan/probabilities.txt', 'r')
    line = file.readline()
    predicates = True
    actions_par_child = False
    while line:
        if line == '-\n':
            predicates = False
        elif predicates:
            split = line.split(' ')
            predicate = split[0]
            spont_false_true = float(split[1])
            spont_true_false = float(split[2].strip('\n'))
            pred_probabilities_map_[predicate] = [spont_false_true, spont_true_false]
        else:
            split = line.split(' ')
            action = split[0]
            action_success = float(split[1])
            effects_success = dict()
            for i in range(2, len(split)):
                predicate = split[i].strip('\n').split('%')[0]
                probability = float(split[i].strip('\n').split('%')[1])
                effects_success[predicate] = probability
            action_probabilities_map_[action] = [action_success, effects_success]
        line = file.readline()
    file.close()


def getElementsFromStateList(elements_list):
    elements_set = set()
    for element in elements_list:
        element_name = element.attribute_name
        for value in element.values:
            element_name = element_name + '#' + value.value
        elements_set.add(element_name)
    return elements_set


def getNodesLayers():
    nodes_layers = list()

    for node in all_nodes_:
        if isPredicate(node):
            nodes_layers.append(node)
    
    rospy.loginfo("Returning nodes layers: " + str(nodes_layers))

    return nodes_layers


# Gets everything needed to start building the network
def setupEverything():
    # print ("Waiting for service")
    rospy.wait_for_service('/rosplan_knowledge_base/domain/operators')
    rospy.wait_for_service('/rosplan_knowledge_base/domain/operator_details')

    # print ("Obtaining operators")
    domain_operators = rospy.ServiceProxy('/rosplan_knowledge_base/domain/operators', GetDomainOperatorService)

    # print('Obtaining goal_')
    global goal_
    goals_list = rospy.ServiceProxy("/rosplan_knowledge_base/state/goals", GetAttributeService)().attributes
    goal_ = getElementsFromStateList(goals_list)

    # print('Obtaining initial state')
    global initial_state_
    initial_state_list = rospy.ServiceProxy('/rosplan_knowledge_base/state/propositions', GetAttributeService)().attributes
    initial_state_ = getElementsFromStateList(initial_state_list)

    # print('Obtaining probabilities')
    getProbabilities()
    # print('   Predicates: ' + str(pred_probabilities_map_))
    # print('   Actions: ' + str(action_probabilities_map_))

    # print('Creating actions')
    operators = domain_operators().operators
    createActions(operators)


######### MAIN FUNCTION #########
## Builds, prunes and writes network to file
def calculateProbability(original_plan):
    # rospy.loginfo("Received plan: " + str(original_plan))
    global all_nodes_

    setupEverything()

    createGroundedActions(original_plan)

    # print("Initial state: " + str(initial_state_))
    # print("goal_: " + str(goal_))
    # print("Original plan: " + str(original_plan))
    # print("Actions: " + str(Action.getActionNamesList()))
    # print("Grounded actions: " + str(GroundedAction.getGroundedActionsList()))

    # Get plan as a list of ActionStart and ActionEnd's instances
    plan = convertPlanToActionStart_End(original_plan)

    predicates_set = set()
    # predicates_par_child is a dictionary where the key is the predicate's name and the value
    ## is another dictionary where the keys are 'parents' and 'children' and the values are sets
    predicates_par_child = dict()
    # same for actions_par_child
    actions_par_child = dict()

    for grounded_action in GroundedAction.actionsList:
        # Adds the predicates in the set grounded_action.getPredicates() to predicates_set
        # Meaning, it adds all needed predicates to predicates_set
        predicates_set |= grounded_action.getPredicates()

    addFirstLayerPredicates(predicates_set, predicates_par_child)

    layer_number = 1
    # Cycle which creates the entire Bayes Network until the end
    for action in plan:

        action_name = action.name + '$' + str(layer_number)

        all_nodes_.append(action_name)
        addActionEdges(action, action_name, predicates_par_child, actions_par_child, layer_number)
        
        # if isinstance(action, ActionStart):
        #     cpd = DiscreteDistribution({'T': success_prob, 'F': 1-success_prob})
        # elif isinstance(action, ActionEnd):
        #     cpd = DiscreteDistribution({'T': 1, 'F': 0})
        # cpds_map_[action_name] = cpd

        
        addPredicateLayer(predicates_set, layer_number, predicates_par_child)

        layer_number = layer_number + 1

    # rospy.loginfo('Predicates parents')
    # rospy.loginfo(getPredicateParentsAsString(predicates_par_child))

    buildCPDs(actions_par_child, predicates_par_child)

    # rospy.loginfo('PRE-PRUNNING')
    # rospy.loginfo(getPredicateParentsAsString(predicates_par_child))

    # print('>>> All nodes: \n' + str(all_nodes_))
    pruneNetwork(actions_par_child, predicates_par_child)

    # model = BayesianNetwork()
    # buildNetworkInModel(model, actions_par_child, predicates_par_child)
    # model.bake()
    # model.plot()
    # plt.show()

    # print('Checking for repeated nodes')
    # checkForRepeatedNodes(all_nodes_)
    # # print('Checking nodes without parents')
    # # checkNodesWithoutParents(parents)
    # # print('Checking nodes without children')
    # # checkNodesWithoutChildren(children, plan)
    # # print('Checking for cycles')
    # # if checkCycles(all_nodes_, children):
    # #     print('  Network has no cycles')
    # # else:
    # #     print('!!! Network has cycles !!! ')
    # print('Writing predicates to file')
    writePredicatesToFile()
    # print('Writing parents to file')
    writePredicateParentsToFile(predicates_par_child)
    # print('Writing actions_par_child to file')
    writeActionsToFile(actions_par_child)
    # print('Writing nodes and CPDs to file')
    writeNodesAndCPDsToFile()
    
    # size = len(plan)
    # distr_dict = dict()
    # for item in all_nodes_:
    #     # If item is part of initial state then set it to true, else false
    #     if len(item.split('%')) > 1:
    #         if item.split('%')[1] == '0':
    #             if item.split('%')[0] in initial_state_:
    #                 distr_dict[item] = 'T'
    #             else:
    #                 distr_dict[item] = 'F'
    # prob_distr = model.predict_proba([distr_dict])[0]

    # goal_distr = dict()
    # for item in goal_:
    #     index = all_nodes_.index(item + '%' + str(size))
    #     goal_distr[all_nodes_[index]] = prob_distr[index]
    
    # print('\n\n>>> goal_ distribution: ')
    # print(goal_distr)
    # TODO: It's just returning a number, it's not calculating the probability yet
    return 0.001

if __name__ == "__main__":
    plan = ['navigate_start#mbot#wp1#wp2#door1#door2', 'navigate_end#mbot#wp1#wp2#door1#door2', 'open_door_start#mbot#wp2#door2', 'open_door_end#mbot#wp2#door2', 'navigate_start#mbot#wp2#wp3#door2#door3', 'navigate_end#mbot#wp2#wp3#door2#door3']
    calculateProbability(plan)