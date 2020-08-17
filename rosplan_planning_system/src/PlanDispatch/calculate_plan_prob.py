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
# predicates_par_child_ is a dictionary where the key is the predicate's name and the value
## is another dictionary where the keys are 'parents' and 'children' and the values are sets
predicates_par_child_ = dict()
# same for actions_par_child_
actions_par_child_ = dict()
layer_number_ = 0

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


        # self.conditions = dict()
        # for k, v in grounded_action.conditions.items():
        #     if k[:8] == 'at_start':
        #         self.conditions[k[9:]] = grounded_action.conditions[k]
        
        # self.effects = dict()
        # for k, v in grounded_action.effects.items():
        #     if k[:8] == 'at_start':
        #         self.effects[k[9:]] = grounded_action.effects[k]

        rospy.loginfo(grounded_action.conditions)

        self.pos_conditions = dict()
        self.neg_conditions = dict()
        for k, v in grounded_action.conditions.items():
            if k == 'at_start_simple_condition':
                self.pos_conditions[k[9:]] = grounded_action.conditions[k]
            elif k == 'at_start_neg_condition':
                self.neg_conditions[k[9:]] = grounded_action.conditions[k]

        self.pos_effects = dict()
        self.neg_effects = dict()
        for k, v in grounded_action.effects.items():
            if k == 'at_start_add_effects':
                self.pos_effects[k[9:]] = grounded_action.effects[k]
            elif k == 'at_start_del_effects':
                self.neg_effects[k[9:]] = grounded_action.effects[k]
        
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
        for k, v in self.pos_conditions.items():
            for predicate in v:
                predicates_set.add('#'.join(predicate))
        
        for k, v in self.neg_conditions.items():
            for predicate in v:
                predicates_set.add('#'.join(predicate))
        
        return predicates_set


    def getPositiveConditionPredicates(self):
        predicates_set = set()
        for k, v in self.pos_conditions.items():
            for predicate in v:
                predicates_set.add('#'.join(predicate))
        
        return predicates_set


    def getNegativeConditionPredicates(self):
        predicates_set = set()
        for k, v in self.neg_conditions.items():
            for predicate in v:
                predicates_set.add('#'.join(predicate))
        
        return predicates_set


    def getEffectsPredicates(self):
        predicates_set = set()
        for k, v in self.effects.items():
            for predicate in v:
                predicates_set.add('#'.join(predicate))
        
        return predicates_set


    def getPositiveEffectsPredicates(self):
        predicates_set = set()
        for k, v in self.pos_effects.items():
            for predicate in v:
                predicates_set.add('#'.join(predicate))
        
        return predicates_set


    def getNegativeEffectsPredicates(self):
        predicates_set = set()
        for k, v in self.neg_effects.items():
            for predicate in v:
                predicates_set.add('#'.join(predicate))
        
        return predicates_set


    def getActionAsString(self):
        s = '\n >>>>>>>>>>>>>>>>>>>>>>>>'
        s = s + '\n >>> ActionStart name: ' + self.name
        s = s + '\n >>> Positive Conditions: ' + str(self.pos_conditions)
        s = s + '\n >>> Negative Conditions: ' + str(self.neg_conditions)
        s = s + '\n >>> Positive Effects: ' + str(self.pos_effects)
        s = s + '\n >>> Negative Effects: ' + str(self.neg_effects)
        s = s + '\n <<<<<<<<<<<<<<<<<<<<<<<<'

        return s


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

        # self.conditions = dict()
        # for k, v in grounded_action.conditions.items():
        #     if k[:6] == 'at_end':
        #         self.conditions[k[7:]] = grounded_action.conditions[k]

        # self.effects = dict()
        # for k, v in grounded_action.effects.items():
        #     if k[:6] == 'at_end':
        #         self.effects[k[7:]] = grounded_action.effects[k]

        # self.over_all_conditions = dict()
        # for k, v in grounded_action.conditions.items():
        #     if k[:8] == 'over_all':
        #         self.over_all_conditions[k[9:]] = grounded_action.conditions[k]


        self.pos_conditions = dict()
        self.neg_conditions = dict()
        for k, v in grounded_action.conditions.items():
            if k == 'at_end_simple_condition':
                self.pos_conditions[k[6:]] = grounded_action.conditions[k]
            elif k == 'at_end_neg_condition':
                self.neg_conditions[k[6:]] = grounded_action.conditions[k]

        self.pos_effects = dict()
        self.neg_effects = dict()
        for k, v in grounded_action.effects.items():
            if k == 'at_end_add_effects':
                self.pos_effects[k[6:]] = grounded_action.effects[k]
            elif k == 'at_end_del_effects':
                self.neg_effects[k[6:]] = grounded_action.effects[k]

        self.pos_over_all_conditions = dict()
        self.neg_over_all_conditions = dict()
        for k, v in grounded_action.conditions.items():
            if k == 'over_all_simple_condition':
                self.pos_over_all_conditions[k[9:]] = grounded_action.conditions[k]
            elif k == 'over_all_neg_condition':
                self.neg_over_all_conditions[k[9:]] = grounded_action.conditions[k]

        ### I don't think over all effects make sense
        # self.over_all_effects = dict()
        # for k, v in grounded_action.effects.items():
        #     if k[:8] == 'over_all':
        #         self.over_all_effects[k[9:]] = grounded_action.effects[k]

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


    def getActionAsString(self):
        s = '\n >>>>>>>>>>>>>>>>>>>>>>>>'
        s = s + '\n >>> ActionStart name: ' + self.name
        s = s + '\n >>> Positive Conditions: ' + str(self.pos_conditions)
        s = s + '\n >>> Negative Conditions: ' + str(self.neg_conditions)
        s = s + '\n >>> Positive Over All Conditions: ' + str(self.pos_over_all_conditions)
        s = s + '\n >>> Negative Over All Conditions: ' + str(self.neg_over_all_conditions)
        s = s + '\n >>> Positive Effects: ' + str(self.pos_effects)
        s = s + '\n >>> Negative Effects: ' + str(self.neg_effects)
        s = s + '\n <<<<<<<<<<<<<<<<<<<<<<<<'

        return s


    def getConditionPredicates(self):
        predicates_set = set()
        for k, v in self.pos_conditions.items():
            for predicate in v:
                predicates_set.add('#'.join(predicate))
        
        for k, v in self.neg_conditions.items():
            for predicate in v:
                predicates_set.add('#'.join(predicate))
        
        return predicates_set


    def getPositiveConditionPredicates(self):
        predicates_set = set()
        for k, v in self.pos_conditions.items():
            for predicate in v:
                predicates_set.add('#'.join(predicate))
        
        return predicates_set


    def getNegativeConditionPredicates(self):
        predicates_set = set()
        for k, v in self.neg_conditions.items():
            for predicate in v:
                predicates_set.add('#'.join(predicate))
        
        return predicates_set


    def getPositiveOverAllPredicates(self):
        predicates_set = set()
        for k, v in self.pos_over_all_conditions.items():
            for predicate in v:
                predicates_set.add('#'.join(predicate))
        
        return predicates_set


    def getNegativeOverAllPredicates(self):
        predicates_set = set()
        for k, v in self.neg_over_all_conditions.items():
            for predicate in v:
                predicates_set.add('#'.join(predicate))
        
        return predicates_set


    def getEffectsPredicates(self):
        predicates_set = set()
        for k, v in self.effects.items():
            for predicate in v:
                predicates_set.add('#'.join(predicate))
        
        return predicates_set


    def getPositiveEffectsPredicates(self):
        predicates_set = set()
        for k, v in self.pos_effects.items():
            for predicate in v:
                predicates_set.add('#'.join(predicate))
        
        return predicates_set


    def getNegativeEffectsPredicates(self):
        predicates_set = set()
        for k, v in self.neg_effects.items():
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


def writePredicateParentsToFile(predicates_par_child_):
    file = open('nodes_parents.txt', 'w')
    file.write('NODES AND PARENTS:\n')
    # For loop to print nodes in order
    for i in range(0, len(predicates_par_child_.keys())):
        for predicate in predicates_par_child_.keys():
            node_index = predicate.split('%')[1]
            if node_index == str(i):
                file.write('>>> Node: ' + predicate + '\n')
                for par in predicates_par_child_[predicate]['parents']:
                    file.write(par + '\n')
                file.write('----------------------------\n')
    file.close()


def getPredicateParentsAsString(predicates_par_child_):
    string = 'NODES AND PARENTS:\n'
    # For loop to print nodes in order
    for i in range(0, len(predicates_par_child_.keys())):
        for predicate in predicates_par_child_.keys():
            node_index = predicate.split('%')[1]
            if node_index == str(i):
                string = string + '>>> Node: ' + predicate + '\n'
                for par in predicates_par_child_[predicate]['parents']:
                    string = string + par + '\n'
                string = string + '----------------------------\n'
    return string


def writeActionsToFile(actions_par_child_):
    file = open('actions_par_child_.txt', 'w')
    file.write('ACTIONS WITH CHILDREN AND PARENTS:\n')
    # This bigger for cycle is to print the actions in order
    for i in range(1, len(actions_par_child_.keys())+1):
        for action_name in actions_par_child_.keys():
            action_index = action_name.split('$')[1]
            if action_index == str(i):
                file.write('>>> ACTION: ' + action_name + '\n')
                file.write('> Parents:\n')
                for par in actions_par_child_[action_name]['parents']:
                    file.write(par + '\n')
                file.write('> Children:\n')
                for child in actions_par_child_[action_name]['children']:
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


def createGroundedAction(action_name):
    name = removeStartEndParamsFromName(action_name)
    if not Action.getAction(name):
        action = Action(name)
    else:
        action = Action.getAction(name)
    return GroundedAction(action, action_name.split('#')[1:])


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
def addPredicate(predicate, layer_numb):
    global all_nodes_
    pred_name = predicate + '%' + str(layer_numb)
    all_nodes_.append(pred_name)
    predicates_par_child_[pred_name] = dict()
    predicates_par_child_[pred_name]['parents'] = set()
    predicates_par_child_[pred_name]['children'] = set()
    if not layer_numb == 0:
        prev_pred_name = predicate + '%' + str(layer_numb-1)
        if prev_pred_name not in all_nodes_:
            addPredicate(prev_pred_name, layer_numb-1)
        predicates_par_child_[pred_name]['parents'].add(prev_pred_name)
        predicates_par_child_[prev_pred_name]['children'].add(pred_name)


def addEffectPredicate(predicate):
    global all_nodes_
    global predicates_par_child_

    all_nodes_.append(predicate)
    predicates_par_child_[predicate] = dict()
    predicates_par_child_[predicate]['parents'] = set()
    predicates_par_child_[predicate]['children'] = set()


def addPredicateLayer(predicates_set):
    for predicate in predicates_set:
        predicate_name = predicate + '%' + str(layer_number_)
        if not predicate_name in all_nodes_:
            addPredicate(predicate, layer_number_)


def addFirstLayerPredicates(predicates_set, predicates_par_child_):
    global all_nodes_
    for predicate in predicates_set:
        pred_name = predicate + '%0'
        all_nodes_.append(pred_name)
        predicates_par_child_[pred_name] = dict()
        predicates_par_child_[pred_name]['parents'] = set()
        predicates_par_child_[pred_name]['children'] = set()


######### CONNECT PREDICATES #########
def connectActionToConditionPredicates(action, action_name):
    global predicates_par_child_
    global actions_par_child_

    for predicate in action.getPositiveConditionPredicates():
        pred_name = predicate + '%' + str(layer_number_)
        predicates_par_child_[pred_name]['children'].add(action_name)
        actions_par_child_[action_name]['pos_parents'].add(pred_name)
    
    for predicate in action.getNegativeConditionPredicates():
        pred_name = predicate + '%' + str(layer_number_)
        predicates_par_child_[pred_name]['children'].add(action_name)
        actions_par_child_[action_name]['neg_parents'].add(pred_name)


def connectActionToEffectsPredicates(action, action_name):
    global all_nodes_
    global predicates_par_child_
    global actions_par_child_
    
    rospy.loginfo(action.getActionAsString())
    for predicate in action.getPositiveEffectsPredicates():
        pred_name = predicate + '%' + str(layer_number_+1)
        addEffectPredicate(pred_name)
        # Connect predicate to action
        predicates_par_child_[pred_name]['parents'].add(action_name)
        actions_par_child_[action_name]['pos_children'].add(pred_name)

    for predicate in action.getNegativeEffectsPredicates():
        pred_name = predicate + '%' + str(layer_number_+1)
        addEffectPredicate(pred_name)
        # Connect predicate to action
        predicates_par_child_[pred_name]['parents'].add(action_name)
        actions_par_child_[action_name]['neg_children'].add(pred_name)


def connectActionEndToActionStart(action, action_name):
    global actions_par_child_

    action_start = action.getActionStart()
    # Search for the layer where actionStart is
    for j in range(1, layer_number_+1):
        action_start_name = action_start.name + '$' + str(j)
        if action_start_name in all_nodes_:
            # Connecting ActionEnd to ActionStart
            actions_par_child_[action_name]['pos_parents'].add(action_start_name)
            actions_par_child_[action_start_name]['pos_children'].add(action_name)
            return j
    return 0


def connectActionToOverAllPredicates(action, action_name, start_index):
    global actions_par_child_
    global predicates_par_child_

    positive_over_all_predicates = action.getPositiveOverAllPredicates()
    negative_over_all_predicates = action.getNegativeOverAllPredicates()

    for j in range(start_index, layer_number_+1):

        for predicate in positive_over_all_predicates:
            pred_name = predicate + '%' + str(j)
            actions_par_child_[action_name]['pos_parents'].add(pred_name)
            predicates_par_child_[pred_name]['children'].add(action_name)

        for predicate in negative_over_all_predicates:  
            pred_name = predicate + '%' + str(j)
            actions_par_child_[action_name]['neg_parents'].add(pred_name)
            predicates_par_child_[pred_name]['children'].add(action_name)


def addActionEdges(action, action_name):
    global actions_par_child_

    actions_par_child_[action_name] = dict()
    actions_par_child_[action_name]['pos_parents'] = set()
    actions_par_child_[action_name]['neg_parents'] = set()
    actions_par_child_[action_name]['pos_children'] = set()
    actions_par_child_[action_name]['neg_children'] = set()

    connectActionToConditionPredicates(action, action_name)
    connectActionToEffectsPredicates(action, action_name)

    if isinstance(action, ActionEnd):
        start_index = connectActionEndToActionStart(action, action_name)
        # If action has no action_start then do not connect to over all predicates
        connectActionToOverAllPredicates(action, action_name, start_index)


######### REMOVE NODE #########
def removeNode(node, actions_par_child_, predicates_par_child_, is_predicate):    
    global all_nodes_
    if is_predicate:
        predicates_par_child_.pop(node)
    else:
        actions_par_child_.pop(node)

    all_nodes_.remove(node)
    # cpds_map_.pop(node)

    for pred in predicates_par_child_.keys():
        if node in predicates_par_child_[pred]['parents']:
            predicates_par_child_[pred]['parents'].remove(node)
        if node in predicates_par_child_[pred]['children']:
            predicates_par_child_[pred]['children'].remove(node)

    for action in actions_par_child_.keys():
            if node in actions_par_child_[action]['parents']:
                actions_par_child_[action]['parents'].remove(node)
            if node in actions_par_child_[action]['children']:
                actions_par_child_[action]['children'].remove(node)


######### PRUNE NETWORK #########
def pruneNetwork(actions_par_child_, predicates_par_child_):
    size = len(actions_par_child_)
    goal_with_time = set()
    for item in goal_:
        goal_with_time.add(item + '%' + str(size))

    for node in reversed(all_nodes_):
        is_predicate = isPredicate(node)

        if is_predicate:


            ##### Rule 1 #####
            # If predicate does not have children and is not the goal_, then remove it
            if not node in goal_with_time:
                if not predicates_par_child_[node]['children']:
                    removeNode(node, actions_par_child_, predicates_par_child_, is_predicate)
                    # Skip to next node in for loop
                    continue


            ##### Rule 3 #####
            ### If predicate is children of action, then remove
            ### edges from other parents and replace CPD
            has_action_as_parent = False
            removed_parents = set()
            for parent in predicates_par_child_[node]['parents']:
                if not isPredicate(parent):
                    has_action_as_parent = True
                    parent_action = parent
                else:
                    removed_parents.add(parent)
            
            if has_action_as_parent:
                for parent in removed_parents:
                    predicates_par_child_[parent]['children'].remove(node)
                    predicates_par_child_[node]['parents'].remove(parent)
                
                action_name_no_time = removeStartEndFromName(parent_action).split('$')[0]
                predicate_no_parameters = node.split('#')[0]
                effects_success = action_probabilities_map_[action_name_no_time][1][predicate_no_parameters]

                cpd = dict()
                cpd['parents'] = predicates_par_child_[node]['parents']
                cpd[(True,)] = effects_success
                cpd[(False,)] = 0
                cpds_map_[node] = cpd


            ##### Rule 2 #####
            ### If predicate is precondition of action then change its CPD to true ###
            # TODO: Think about how to implement this later
            # Maybe implement it in the formula that's calculating the probability

            # We don't want to change the CPDs of predicates in the first layer
            # if not node.split('%')[1] == str(0):

            #     has_action_as_child = False
            #     for child in predicates_par_child_[node]['children']:
            #         if not isPredicate(child):
            #             has_action_as_child = True
            #             child_action = child
            #             break

            #     if has_action_as_child:

            #         # Get all combinations of 'F' and 'T' has a list of len(actions_par_child_[node]['parents'])+1 items
            #         # I add 1 because the last entry represents the node itself
            #         cpd_lines = list(itertools.product(['F', 'T'], repeat=len(predicates_par_child_[node]['parents'])))

            #         cpd = dict()
            #         cpd['parents'] = predicates_par_child_[node]['parents']
            #         cpd['value'] = 1
            #         cpd_elements = list()

            #         parents_cpd_list = list()
            #         for predicate in predicates_par_child_[node]['parents']:
            #             parents_cpd_list.append(cpds_map_[predicate])
            #         parents_cpd_list.append(cpds_map_[parent_action])

            #         for line in cpd_lines:
            #             lst = list(line)
            #             if all([elem == 'T' for elem in lst]):
            #                 lst.append(1)
            #             elif all([elem == 'T' for elem in lst[:-1]]):
            #                 lst.append(0)
            #             elif lst[-1] == 'T':
            #                 lst.append(0)
            #             else:
            #                 lst.append(1)
            #             cpd_elements.append(lst)      
                    
            #         cpds_map_[node] = cpd


######### BUILD NETWORK IN MODEL #########
def buildNetworkInModel(model, actions_par_child_, predicates_par_child_):
    nodes_dict = dict()
    for node_name in all_nodes_:        
        cpd = cpds_map_[node_name]
        node = Node(cpd, name=node_name)
        nodes_dict[node_name] = node
        model.add_node(node)

        if isPredicate(node_name):
            parents_dict = predicates_par_child_
        else:
            parents_dict = actions_par_child_
      
        for parent in parents_dict[node_name]['parents']:
            parent_node = nodes_dict[parent]
            model.add_edge(parent_node, node)


######### BUILD CPDs #########
def buildCPDs():

    nodes_without_cpds = set(all_nodes_) - set(cpds_map_.keys())

    for node in nodes_without_cpds:

        if isPredicate(node):

            node_without_time = node.split('%')[0]

            # If node is in first layer
            if int(node.split('%')[1]) == 0:
                # If it is in initial state then node is true
                if node.split('%')[0] in initial_state_:
                    cpds_map_[node] = 1
                    continue
                # Else it is false
                cpds_map_[node] = 0
                continue

            # If predicate only has one parent, which can only be
            # the same predicate in the previous layer
            if len(predicates_par_child_[node]['parents']) == 1:
                spont_false_true = pred_probabilities_map_[node_without_time][0]
                spont_true_false = pred_probabilities_map_[node_without_time][1]
                cpd = dict()
                cpd['parents'] = predicates_par_child_[node]['parents']
                # CPD's Key is the value of parent and its Value is the probability for True
                cpd[(True,)] = 1-spont_true_false
                cpd[(False,)] = spont_false_true
                cpds_map_[node] = cpd
                continue

            '''
            # If predicate has more than one parent, then it has two:
            # the predicate in the previous layer and an action
            else:

                spont_false_true = pred_probabilities_map_[node_without_time][0]
                spont_true_false = pred_probabilities_map_[node_without_time][1]

                action_name_without_time = removeStartEndFromName(parent_action).split('$')[0]
                predicate_without_parameters = node.split('#')[0]
                effects_success = action_probabilities_map_[action_name_without_time][1][predicate_without_parameters]

                i = 0
                action_index = 0
                for parent in predicates_par_child_[node]['parents']:
                    if not isPredicate(parent):
                        action_index = i
                    else:
                        predicate_index = i
                    i = i + 1

                cpd = dict()

                if action_index == 0:
                    cpd[(False, False)] = spont_false_true
                    cpd[(False, True)] = 1-spont_true_false
                    cpd[(True, False)] = effects_success
                    cpd[(True, True)] = effects_success
                else:
                    cpd[(False, False)] = spont_false_true
                    cpd[(False, True)] = effects_success
                    cpd[(True, False)] = 1-spont_true_false
                    cpd[(True, True)] = effects_success

                cpd['parents'] = predicates_par_child_[node]['parents']
                cpds_map_[node] = cpd
            '''

        # If node is an action
        else:

            is_at_end_action = False
            if node.split('#')[0][-3:] == 'end':
                is_at_end_action = True

            number_of_pos_precond = len(actions_par_child_[node]['pos_parents'])
            number_of_neg_precond = len(actions_par_child_[node]['neg_parents'])
            number_of_precond = number_of_pos_precond + number_of_neg_precond

            cpd_lines = tuple(itertools.product([False, True], repeat=number_of_precond))

            action_name_without_time = removeStartEndFromName(node).split('$')[0]
            success_prob = action_probabilities_map_[action_name_without_time][0]
            
            cpd = dict()
            for i in range(len(cpd_lines)):
                lst = cpd_lines[i]
                # If action has both positive and negative preconditions
                if not number_of_neg_precond == 0 and not number_of_pos_precond == 0:
                    if all([elem == True for elem in lst[:number_of_pos_precond]]) and all([elem == False for elem in lst[-number_of_neg_precond:]]):
                        if is_at_end_action:
                            cpd[lst] = 1
                        else:
                            cpd[lst] = success_prob
                    else:
                        cpd[lst] = 0
                # If action only has positive preconditions
                elif number_of_neg_precond == 0:
                    if all([elem == True for elem in lst]):
                        if is_at_end_action:
                            cpd[lst] = 1
                        else:
                            cpd[lst] = success_prob
                    else:
                        cpd[lst] = 0
                # If action only has negative preconditions
                else:
                    if all([elem == False for elem in lst]):
                        if is_at_end_action:
                            cpd[lst] = 1
                        else:
                            cpd[lst] = success_prob
                    else:
                        cpd[lst] = 0

            cpd['parents'] = actions_par_child_[node]['pos_parents'].union(actions_par_child_[node]['neg_parents'])

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


def convertActionToActionStart_End(received_action):
    name = removeStartEndFromName(received_action)
    grounded_action = GroundedAction.getGroundedAction(name)

    if received_action.split('#')[0][-5:] == 'start':
        action = ActionStart(grounded_action)
    elif received_action.split('#')[0][-3:] == 'end':
        action = ActionEnd(grounded_action)
    else:
        rospy.logerr('ERROR: NULL WHEN CONVERTING ACTION TO ACTION START/END -> ' + received_action)
    
    return action


######### GET STUFF #########
def getProbabilities():
    global pred_probabilities_map_
    global action_probabilities_map_
    
    file = open('/home/tomas/ros_ws/src/ROSPlan/src/rosplan/probabilities.txt', 'r')
    line = file.readline()
    predicates = True
    actions_par_child_ = False
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


'''
def nodeHasParentToBeMarginalised(node, nodes_to_be_marginalised, predicates_par_child_):
    for parent in predicates_par_child_[node]['parents']:
        if parent in nodes_to_be_marginalised:
            return True
    return False


def calculateSuccessProbability(actions_par_child_, predicates_par_child_, max_layer_number):
    for goal_fact in goal_:
        fact = goal_fact + '%' + str(max_layer_number)

    ### Choose nodes to be marginalised ###
    # A node does not need to be marginalised if:
        # It's in the first or last layer
        # Is a precondition of an action and its parents aren't variables to be marginalised


    nodes_to_be_marginalised = set()
    variables_to_marginalise = set()
    for node in all_nodes_:
        if isPredicate(node):
            layer_number_ = int(node.split('%')[1])
            if not layer_number_ == 0 and not layer_number_ == max_layer_number:
                if not nodeIsPreconditionOfAction(node, predicates_par_child_):
                    nodes_to_be_marginalised.add(node)
                    variables_to_marginalise.add(node)
                    continue
                elif nodeHasParentToBeMarginalised(node, nodes_to_be_marginalised, predicates_par_child_) and isPredicate(node):
                    nodes_to_be_marginalised.add(node)
                else

    # Set of nodes that can be directly calculated
    direct_calc_nodes = all_nodes - nodes_to_be_marginalised

    # Calculate probability
    probability = 1.0
    nodes_already_calculated_ = set()

    for node in direct_calc_nodes:

            if isPredicate(node):
                node_without_time = node.split('%')[0]
                layer_number_ = int(node.split('%')[1])
                predicate_without_params = node.split('#')[0]

                # If node is in the initial_state_
                if node_without_time in initial_state_:
                    probability = probability*cpds_map_[node]
                    continue

                # If node is not in initial_state_ but is in first layer
                elif layer_number_ == 0:
                    probability = probability*(1-cpds_map_[node]['false'])
                    continue

                    
                # If is precondition of an action
                positive_precond = False
                for child in predicates_par_child_[node]['children']:
                    if not isPredicate(child):
                        if node in actions_par_child_[child]['pos_parents']:
                            positive_precond = True
                
                if 

                # When node to be marginalised is false
                
                # If node is part of the goal
                

            ## If node is an action
            else:
                probability = probability*cpds_map_[node]
                continue

                # # If node is precondition of action
                # child_actions = set()
                # for child in predicates_par_child_[node]['children']:
                #     if not isPredicate(child):
                #         child_actions.add(child)

                # for child in child_actions:
                #     if node in actions_par_child_[child]['pos_parents']:
                #         probability = probability*cpds_map_[node]['true']
                #         continue
                #     elif node in actions_par_child_[child]['neg_parents']:
                #         probability = probability*cpds_map_[node]['false']
                #         continue
'''


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

    for grounded_action in GroundedAction.actionsList:
        # Adds the predicates in the set grounded_action.getPredicates() to predicates_set
        # Meaning, it adds all needed predicates to predicates_set
        predicates_set |= grounded_action.getPredicates()

    addFirstLayerPredicates(predicates_set, predicates_par_child_)

    layer_number_ = 1
    # Cycle which creates the entire Bayes Network until the end
    for action in plan:

        action_name = action.name + '$' + str(layer_number_)

        all_nodes_.append(action_name)
        addActionEdges(action, action_name, predicates_par_child_, actions_par_child_, layer_number_)
        
        # if isinstance(action, ActionStart):
        #     cpd = DiscreteDistribution({'T': success_prob, 'F': 1-success_prob})
        # elif isinstance(action, ActionEnd):
        #     cpd = DiscreteDistribution({'T': 1, 'F': 0})
        # cpds_map_[action_name] = cpd

        
        addPredicateLayer(predicates_set, layer_number_, predicates_par_child_)

        layer_number_ = layer_number_ + 1

    # rospy.loginfo('Predicates parents')
    # rospy.loginfo(getPredicateParentsAsString(predicates_par_child_))

    buildCPDs(actions_par_child_, predicates_par_child_)

    # rospy.loginfo('PRE-PRUNNING')
    # rospy.loginfo(getPredicateParentsAsString(predicates_par_child_))

    # print('>>> All nodes: \n' + str(all_nodes_))
    pruneNetwork(actions_par_child_, predicates_par_child_)

    # calculateSuccessProbability(actions_par_child_, predicates_par_child_, max_layer_number)

    # model = BayesianNetwork()
    # buildNetworkInModel(model, actions_par_child_, predicates_par_child_)
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
    writePredicateParentsToFile(predicates_par_child_)
    # print('Writing actions_par_child_ to file')
    writeActionsToFile(actions_par_child_)
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


def getTopPredicateParent(node):
    # This is just so it passes the first check of isPredicate
    # TODO: Check this try
    parent = node
    while isPredicate(parent):
        previous_parent = parent
        parents_set = predicates_par_child_[parent]['parents']
        if len(parents_set) > 0:
            parent = parents_set.pop()
        else:
            return previous_parent


def getPredicateChild(node):
    for child in predicates_par_child_[node]['children']:
            if isPredicate(child):
                return child


def calculateColumn(bottom_node, true_value):
    rospy.loginfo('>>> Inside calculateColumn <<<')
    top_parent = getTopPredicateParent(bottom_node)
    rospy.loginfo('Top parent: ' + top_parent)

    if len(predicates_par_child_[top_parent]['parents']) == 0:
        prob = cpds_map_[top_parent]
    else:
        prob = cpds_map_[top_parent][(True,)]

    node = top_parent

    while not node == bottom_node:
        node = getPredicateChild(node)
        spont_true_false = cpds_map_[node][(True,)]
        spont_false_true = cpds_map_[node][(False,)]
        
        prob = prob*(1-spont_true_false) + (1-prob)*spont_false_true

    if true_value:
        returned_prob = prob
    else:
        returned_prob = 1-prob
    
    rospy.loginfo('>>> Exiting calculateColumn with prob: ' + str(returned_prob) + ' <<<')
    return returned_prob


def calculateActionsProbability(prob, action_name):

    rospy.loginfo('*** Inside calculateActionsProbability ***')

    number_parents_action = len(cpds_map_[action_name]['parents'])
    rospy.loginfo('Number of parents of action: ' + str(number_parents_action))
    all_true = (True,)*number_parents_action
    rospy.loginfo('All true: ' + str(all_true))
    action_success_prob = cpds_map_[action_name][all_true]
    rospy.loginfo('Action success probability: ' + str(action_success_prob))
    # rospy.loginfo('Action CPD: ' + str(cpds_map_[action_name]))
    prob = prob*action_success_prob
    rospy.loginfo('Probability after action: ' + str(prob))

    rospy.loginfo('Checking positive parents')
    for parent in actions_par_child_[action_name]['pos_parents']:
        rospy.loginfo('Parent: ' + parent)
        if isPredicate(parent):
            rospy.loginfo('Calculating column of positive parent')
            prob = prob*calculateColumn(parent, True)
            rospy.loginfo('Probability after: ' + str(prob))
    
    rospy.loginfo('Checking negative parents')
    for parent in actions_par_child_[action_name]['neg_parents']:
        rospy.loginfo('Parent: ' + parent)
        if isPredicate(parent):
            rospy.loginfo('Calculating column of negative parent')
            prob = prob*calculateColumn(parent, False)
            rospy.loginfo('Probability after: ' + str(prob))
    
    rospy.loginfo('*** Exiting calculateActionsProbability with probability ' + str(prob) + ' ***')
    return prob


def func(received_action_name, prob):
    global layer_number_

    rospy.loginfo('Setting up')
    setupEverything()

    rospy.loginfo('Creating grounded action')
    grounded_action = createGroundedAction(received_action_name)
    rospy.loginfo('Converting grounded action to action start/end')
    action = convertActionToActionStart_End(received_action_name)

    rospy.loginfo('Getting condition predicates')
    predicates_set = action.getConditionPredicates()
    rospy.loginfo('>>> Predicates set: ' + str(predicates_set))

    rospy.loginfo('Adding predicate layer')
    addPredicateLayer(predicates_set)

    action_name = action.name + '$' + str(layer_number_+1)

    all_nodes_.append(action_name)

    rospy.loginfo('>>> Adding action edges')
    addActionEdges(action, action_name)

    rospy.loginfo('Building CPDs')
    buildCPDs()

    rospy.loginfo('Calculating probability')
    prob = calculateActionsProbability(prob, action_name)

    layer_number_ = layer_number_ + 1

    rospy.loginfo('Returning probability: ' + str(prob))
    return prob


if __name__ == "__main__":
    plan = ['navigate_start#mbot#wp1#wp2#door1#door2', 'navigate_end#mbot#wp1#wp2#door1#door2', 'open_door_start#mbot#wp2#door2', 'open_door_end#mbot#wp2#door2', 'navigate_start#mbot#wp2#wp3#door2#door3', 'navigate_end#mbot#wp2#wp3#door2#door3']
    # calculateProbability(plan)
    setupEverything()
    prob = 1
    for action in plan:
        prob = prob*func(action, prob)
        print('>>> Probability: ' + str(prob))