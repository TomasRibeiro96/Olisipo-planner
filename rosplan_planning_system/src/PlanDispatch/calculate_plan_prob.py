#!/usr/bin/env python

import sys
import rospy
from rosplan_knowledge_msgs.srv import *
from rosplan_knowledge_msgs.msg import *
from rosplan_dispatch_msgs.msg import EsterelPlanArray
from std_msgs.msg import String
import collections
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

# Dictionary that connects the node to the true probability of its column above
prob_node_column_ = dict()

list_probabilities_ = list()

joint_prob_ = 1

accounted_nodes_ = set()

# List of nodes added on each
added_nodes_ = list()

rospy.init_node('bayesian_network_calculator')


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

        predicates_set |= self.getPositiveConditionPredicates()
        predicates_set |= self.getNegativeConditionPredicates()
        
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

        predicates_set |= self.getPositiveEffectsPredicates()
        predicates_set |= self.getNegativeEffectsPredicates()
        
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

        predicates_set |= self.getPositiveConditionPredicates()
        predicates_set |= self.getNegativeConditionPredicates()
        predicates_set |= self.getPositiveOverAllPredicates()
        predicates_set |= self.getNegativeOverAllPredicates()
        
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

        predicates_set |= self.getPositiveEffectsPredicates()
        predicates_set |= self.getNegativeEffectsPredicates()
        
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


def isActionEnd(node):
    if node.split('#')[0][-3:] == 'end':
        return True
    else:
        return False


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


def removeStartEndAndParamsFromName(name):
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


def writePredicateParentsToFile():
    file = open('nodes_parents.txt', 'w')
    file.write('NODES AND PARENTS:\n')
    # For loop to print nodes in order
    # for i in range(0, len(predicates_par_child_.keys())):
    #     for predicate in predicates_par_child_.keys():
    #         node_index = predicate.split('%')[1]
    #         if node_index == str(i):
    #             file.write('>>> Node: ' + predicate + '\n')
    #             for par in predicates_par_child_[predicate]['parents']:
    #                 file.write(par + '\n')
    #             file.write('----------------------------\n')

    for layer_index in range(layer_number_):
        for node in all_nodes_:
            if isPredicate(node):
                node_index = int(node.split('%')[1])
                if layer_index == node_index:
                    file.write('>>> Node: ' + node + '\n')
                    for parent in predicates_par_child_[node]['parents']:
                        file.write(parent + '\n')
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
        if isinstance(cpds_map_[node], int):
            file.write(str(cpds_map_[node]))
        else:
            for k, v in cpds_map_[node].items():
                file.write(str(k) + ': ' + str(v) + '\n')
        file.write('\n')
    file.close()


def writeNodesAndCPDsToFileForBayesnet():
    file = open('bayesnetinference/nodes_cpds.bn', 'w')
    # file.write('NODES AND CPDs:\n')
    for node in all_nodes_:
        if isinstance(cpds_map_[node], int):
            file.write('P(' + node + ') = ' + str(float(cpds_map_[node])) + '\n\n')
        else:
            if isinstance(cpds_map_[node]['parents'], set):
                for parent in cpds_map_[node]['parents']:
                    file.write(parent + ' ')
            else:
                file.write(cpds_map_[node]['parents'] + ' ')
            file.write('| ' + node + '\n')
            number_parents = len(cpds_map_[node]['parents'])
            separation_line = '--'*number_parents + '|-----\n'
            file.write(separation_line)
            for line in cpds_map_[node].keys():
                if isinstance(line, tuple):
                    for elem in line:
                        if elem == True:
                            file.write('t ')
                        else:
                            file.write('f ')
                    file.write('| ' + str(float(cpds_map_[node][line])) + '\n')
            file.write('\n')
        # file.write('>>> Node: ' + node + '\n')
        # file.write(str(cpds_map_[node]) + '\n')
    file.close()


######### CREATE ACTIONS #########
def createActions(operators):
    ''' Creates action instances for all actions in domain file '''
    for i in range(len(operators)):
        action_name = operators[i].name
        Action(action_name)


def createGroundedActions(original_plan):
    for action_name in original_plan:
        name = removeStartEndAndParamsFromName(action_name)
        if not Action.getAction(name):
            action = Action(name)
        else:
            action = Action.getAction(name)
        GroundedAction(action, action_name.split('#')[1:])


def createGroundedAction(action_name):
    name = removeStartEndAndParamsFromName(action_name)
    if not Action.getAction(name):
        action = Action(name)
    else:
        action = Action.getAction(name)
    return GroundedAction(action, action_name.split('#')[1:])


######### ADD PREDICATES #########
def addPredicate(predicate, layer_numb):
    global all_nodes_
    global added_nodes_

    pred_name = predicate + '%' + str(layer_numb)
    all_nodes_.append(pred_name)
    added_nodes_[layer_numb].add(pred_name)

    predicates_par_child_[pred_name] = dict()
    predicates_par_child_[pred_name]['parents'] = set()
    predicates_par_child_[pred_name]['children'] = set()

    # rospy.loginfo('Adding predicate: ' + pred_name)
    # rospy.loginfo('Layer number: ' + str(layer_numb))

    if not layer_numb == 0:
        prev_pred_name = predicate + '%' + str(layer_numb-1)
        # rospy.loginfo('Previous predicate: ' + str(prev_pred_name))
        if prev_pred_name not in all_nodes_:
            # rospy.loginfo('Adding previous predicate to all_nodes_')
            addPredicate(predicate, layer_numb-1)
        
        # rospy.loginfo('Connecting to previous predicate')
        predicates_par_child_[pred_name]['parents'].add(prev_pred_name)
        predicates_par_child_[prev_pred_name]['children'].add(pred_name)


def addEffectPredicate(predicate):
    global added_nodes_

    ''' Here we don't add the predicate in previous layers because
        its CPD will only depend on the action that it's its parent
    '''
    global all_nodes_
    global predicates_par_child_

    all_nodes_.append(predicate)
    added_nodes_[layer_number_].add(predicate)

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


######### BUILD CPDs #########
def buildCPDs():

    global cpds_map_

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

            # Each node should only have one or zero parents, this elif is just a safety measure
            elif len(predicates_par_child_[node]['parents']) == 1:

                parent = next(iter(predicates_par_child_[node]['parents']))
                # If predicate has another predicate as parent
                if isPredicate(parent):
                    spont_false_true = pred_probabilities_map_[node_without_time][0]
                    spont_true_false = pred_probabilities_map_[node_without_time][1]
                    cpd = dict()
                    cpd['parents'] = predicates_par_child_[node]['parents']
                    # CPD's Key is the value of parent and its Value is the probability for True
                    cpd[(True,)] = 1-spont_true_false
                    cpd[(False,)] = spont_false_true
                    cpds_map_[node] = cpd
                    continue
                
                # If predicate has an action has parent
                else:
                    # rospy.loginfo('>>>>> Predicate with action as parent: ' + str(node) + '<<<<<<<')
                    cpd = dict()
                    action_without_time = removeStartEndFromName(parent).split('$')[0]
                    predicate_without_time_and_params = node.split('%')[0].split('#')[0]
                    cpd[(True,)] = action_probabilities_map_[action_without_time][1][predicate_without_time_and_params]
                    cpd[(False,)] = 0
                    cpd['parents'] = parent
                    # rospy.loginfo('>>>> CPT: ' + str(cpd))
                    cpds_map_[node] = cpd


        # If node is an action
        else:

            is_at_end_action = isActionEnd(node)

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
    orderAllNodes()
    nodes_layers = list()

    for node in all_nodes_:
        if isPredicate(node):
            nodes_layers.append(node)
    
    # rospy.loginfo("Returning nodes layers: " + str(nodes_layers))

    return nodes_layers


# Gets everything needed to start building the network
def setup():
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


def getTopPredicateParent(node):
    # rospy.loginfo('\t\t\t\t\t*** Inside getTopPredicateParent ***')

    return_previous_parent = False
    while True:
        if node in accounted_nodes_:
            return node
        previous_node = node
        parents_set = predicates_par_child_[node]['parents']
        # If node has parents then keep searching up, unless the parent is an action
        return_previous_node = False
        if len(parents_set) > 0:
            node = next(iter(parents_set))
            if not isPredicate(node):
                return_previous_node = True
        else:
            return_previous_node = True

        if return_previous_node:
            # rospy.loginfo('\t\t\t\t\t*** Exiting getTopPredicateParent: ' + previous_parent + ' ***')
            return previous_node


def getPredicateChild(node):
    for child in predicates_par_child_[node]['children']:
            if isPredicate(child):
                return child


def predicateHasActionAsChild(node):
    for child in predicates_par_child_[node]['children']:
        if not isPredicate(child):
            return child
    return None


def predicateHasActionAsParent(node):
    for parent in predicates_par_child_[node]['parents']:
        if not isPredicate(parent):
            return parent
    return None


def parentHasActionAsChild(node):
    parent = next(iter(predicates_par_child_[node]['parents']))

    return predicateHasActionAsChild(parent)


def calculateColumn(bottom_node, true_value):
    global prob_node_column_
    global accounted_nodes_

    # rospy.loginfo('\t\t\t\t>>> Inside calculateColumn <<<')
    # rospy.loginfo('\t\t\t\tBottom Node: ' + bottom_node)
    # rospy.loginfo('\t\t\t\tNode value: ' + str(true_value))

    top_parent = getTopPredicateParent(bottom_node)

    # If top_parent is inside already calculated block,
    # then don't account for it
    if top_parent in accounted_nodes_:
        prob = 1
    else:
        prob = cpds_map_[top_parent]
        # If prob has not been calculated and it's a
        # dictionary, then it has an action as parent
        if isinstance(prob, dict):
            prob = prob[(True,)]
            accounted_nodes_.add(top_parent)


    # rospy.loginfo('\t\t\t\tTop parent: ' + top_parent)
    # rospy.loginfo('\t\t\t\tTop Parent Probability: ' + str(prob))

    node = top_parent

    # rospy.loginfo('\t\t\t\tStarting to calculate from top node to bottom node')


    while not node == bottom_node:
        node = getPredicateChild(node)

        spont_true_false = 1-cpds_map_[node][(True,)]
        spont_false_true = cpds_map_[node][(False,)]

        # rospy.loginfo('\t\t\t\t\tNode: ' + node)
        # rospy.loginfo('\t\t\t\t\tSpont_true_false: ' + str(spont_true_false))
        # rospy.loginfo('\t\t\t\t\tSpont_false_true: ' + str(spont_false_true))
        # rospy.loginfo('\t\t\t\t\tNode CPD: ' + str(cpds_map_[node]))

        if parentHasActionAsChild(node):
            # rospy.loginfo('\t\t\t\t\tFactor Probability (only true): ' + str(prob*(1-spont_true_false)))
            prob = prob*(1-spont_true_false)
        else:
            # rospy.loginfo('\t\t\t\t\tFactor Probability (true and false): ' + str(prob*(1-spont_true_false) + (1-prob)*spont_false_true))
            prob = prob*(1-spont_true_false) + (1-prob)*spont_false_true

        accounted_nodes_.add(node)


    if true_value:
        returned_prob = prob
    else:
        returned_prob = 1-prob

    accounted_nodes_.add(bottom_node)
    # rospy.loginfo('\t\t\t\t>>> Exiting calculateColumn with prob: ' + str(returned_prob) + ' <<<')
    return returned_prob


def calculateActionsJointProbability(action_name):
    global joint_prob_
    global added_nodes_
    global accounted_nodes_

    # rospy.loginfo('\t*** Inside calculateActionsProbability ***')

    prob = 1

    number_parents_action = len(cpds_map_[action_name]['parents'])
    # rospy.loginfo('\tNumber of parents of action: ' + str(number_parents_action))
    all_true = (True,)*number_parents_action
    # rospy.loginfo('\tAll true: ' + str(all_true))
    action_success_prob = cpds_map_[action_name][all_true]
    # rospy.loginfo('\tAction success probability: ' + str(action_success_prob))
    # rospy.loginfo('Action CPD: ' + str(cpds_map_[action_name]))
    prob = prob*action_success_prob
    # rospy.loginfo('\tProbability after action: ' + str(prob))

    # rospy.loginfo('\tChecking positive parents')
    for parent in actions_par_child_[action_name]['pos_parents']:
        # rospy.loginfo('\t\tParent: ' + parent)
        if isPredicate(parent):
            # rospy.loginfo('\t\t\tCalculating column of positive parent')
            prob = prob*calculateColumn(parent, True)
            # rospy.loginfo('\t\t\tProbability after: ' + str(prob))

    # rospy.loginfo('\tChecking negative parents')
    for parent in actions_par_child_[action_name]['neg_parents']:
        # rospy.loginfo('\t\tParent: ' + parent)
        if isPredicate(parent):
            # rospy.loginfo('\t\t\tCalculating column of negative parent')
            prob = prob*calculateColumn(parent, False)
            # rospy.loginfo('\t\t\tProbability after: ' + str(prob))
    
    accounted_nodes_.add(action_name)
    
    joint_prob_ = joint_prob_*prob
    # rospy.loginfo('\t*** Exiting calculateActionsProbability with probability ' + str(joint_prob_) + ' ***')


def orderAllNodes():
    global all_nodes_
    ordered_nodes = list()

    i = 0
    index_has_predicates = True

    while index_has_predicates:
        index_has_predicates = False
        for node in all_nodes_:
            if len(node.split('%')) > 1:
                if int(node.split('%')[1]) == i:
                    ordered_nodes.append(node)
                    index_has_predicates = True
            elif len(node.split('$')) > 1:
                if int(node.split('$')[1]) == i:
                    ordered_nodes.append(node)
                    index_has_predicates = True
        i = i + 1
        if not index_has_predicates:
            break

    all_nodes_ = ordered_nodes


def writeBayesNetAIMAToFile():
    file = open('bayesNetAIMA.txt', 'w')
    for node in all_nodes_:
        file.write('-\n')
        file.write(node + '\n')
        if isPredicate(node):
            file.write(str(list(predicates_par_child_[node]['parents'])) + '\n')
        else:
            parents = list(actions_par_child_[node]['pos_parents'].union(actions_par_child_[node]['neg_parents']))
            file.write(str(parents) + '\n')
        
        cpt = cpds_map_[node]
        if isinstance(cpt, dict):
            cpt.pop('parents')
        file.write(str(cpt) + '\n')
    file.close()


def getActionsJointProbability(received_action_name):
    global layer_number_
    global added_nodes_

    # Need to convert this to handle backtracking
    if layer_number_ == len(added_nodes_):
        added_nodes_.append(set())

    # rospy.loginfo('Action: ' + received_action_name)

    # rospy.loginfo('Creating grounded action')
    grounded_action = createGroundedAction(received_action_name)
    # rospy.loginfo('Converting grounded action to action start/end')
    action = convertActionToActionStart_End(received_action_name)

    # rospy.loginfo('Getting condition predicates')
    predicates_set = action.getConditionPredicates()

    # rospy.loginfo('Adding predicate layer')
    addPredicateLayer(predicates_set)

    action_name = action.name + '$' + str(layer_number_+1)

    all_nodes_.append(action_name)
    added_nodes_[layer_number_].add(action_name)

    # rospy.loginfo('Adding action edges')
    addActionEdges(action, action_name)

    # rospy.loginfo('Building CPDs')
    buildCPDs()

    # rospy.loginfo('Calculating probability')
    calculateActionsJointProbability(action_name)

    layer_number_ = layer_number_ + 1

    list_probabilities_.append(joint_prob_)

    # rospy.loginfo(' Returning probability: ' + str(joint_prob_))

    return joint_prob_


def getProbFromBayesNetAIMA():
    prob_list = list()
    file = open('prob_bayesNetAIMA.txt', 'r')

    line = file.readline()

    while line != '':
        prob_list.append(float(line))
        line = file.readline()

    return prob_list


def calculateFullJointProbability():
    global joint_prob_

    for node in goal_:
        goal_node = node + '%' + str(layer_number_)

        if goal_node not in all_nodes_:
            addPredicate(node, layer_number_)
        
        buildCPDs()

        factor = calculateColumn(goal_node, True)

        joint_prob_ = factor*joint_prob_

        list_probabilities_.append(joint_prob_)

    # rospy.loginfo('Returning probability: ' + str(joint_prob_))
    return joint_prob_


''' This is only for testing with the 'skip_door' domain and 'problem_r1_w3' '''
if __name__ == "__main__":
    plan = ['navigate_start#mbot#wp1#wp2#door1#door2', 'navigate_end#mbot#wp1#wp2#door1#door2', 'open_door_start#mbot#wp2#door2', 'open_door_end#mbot#wp2#door2', 'navigate_start#mbot#wp2#wp3#door2#door3', 'navigate_end#mbot#wp2#wp3#door2#door3']    
    list_actions = ['navigate_start#mbot#wp1#wp2#door1#door2$1', 'navigate_end#mbot#wp1#wp2#door1#door2$2', 'open_door_start#mbot#wp2#door2$3', 'open_door_end#mbot#wp2#door2$4', 'navigate_start#mbot#wp2#wp3#door2#door3$5', 'navigate_end#mbot#wp2#wp3#door2#door3$6']
    list_goal = ['robot_at#mbot#wp3%6']

    list_actions_goal = list_actions + list_goal

    rospy.loginfo('Setting up')
    setup()
    for action in plan:
        getActionsJointProbability(action)

    calculateFullJointProbability()

    orderAllNodes(plan)
    writeNodesAndCPDsToFile()

    # AIMA - BayesNet    
    writeBayesNetAIMAToFile()
    bayes_net_AIMA = getProbFromBayesNetAIMA()

    for i in range(len(list_actions_goal)):
        node = list_actions_goal[i]+'$'+str(i+1)

        rospy.loginfo('>>> Node: ' + node)
        rospy.loginfo('Code probability:           ' + str(round(list_probabilities_[i], 6)))
        rospy.loginfo('Bayes net AIMA probability: ' + str(round(bayes_net_AIMA[i], 6)) + '\n')

    writePredicatesToFile()
    writePredicateParentsToFile()