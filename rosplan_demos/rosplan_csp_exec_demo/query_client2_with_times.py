#!/usr/bin/env python

import sys
import rospy
from rosplan_knowledge_msgs.srv import *
from rosplan_knowledge_msgs.msg import *
from rosplan_dispatch_msgs.msg import EsterelPlanArray
from rosplan_dispatch_msgs.msg import EsterelPlan
import matplotlib
import matplotlib.pyplot as plt
from numpy import *
from pomegranate import *



class Action:

    # Class parameter with all actions
    actionsList = list()

    def __init__(self, name, duration = 0):
        if not isinstance(name, str):
            raise TypeError("Action name not a String")
        self.name = name
        self.duration = duration

        self.parameters = list()
        operator_details = rospy.ServiceProxy('/rosplan_knowledge_base/domain/operator_details', GetDomainOperatorDetailsService)(name)
        rospy.ServiceProxy('/rosplan_knowledge_base/domain/operator_details', GetDomainOperatorDetailsService)
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



class GroundedAction:

    actionsList = list()

    def __init__(self, action, list_objects):
        # Action duration is missing
        variables = action.parameters
        var_obj = dict(zip(variables, list_objects))
        self.name = str( action.name + '#' + '#'.join(str(v) for v in list_objects))

        self.effects = dict()
        for k, v in action.effects.items():
            self.effects[k] = list()
            if v:
                i = 0
                for value in v:
                    self.effects[k].append(list())
                    for item in value:
                        if item in list(var_obj.keys()):
                            self.effects[k][i].append(var_obj[item])
                        else:
                            self.effects[k][i].append(item)
                    i = i + 1
            else:
                self.effects[k] = v
        
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


    def printAction(self):
        print('>>>>>>>>>>>>>>>>>>>>>>>>')
        print('Grounded Action name: ' + self.name)
        # print('Action duration: ' + str(self.duration))
        print('Conditions: ' + str(self.conditions))
        print('Effects: ' + str(self.effects))
        print('Comparison: ' + str(self.comparison))
        print('<<<<<<<<<<<<<<<<<<<<<<<<')


    @classmethod
    def getGroundedAction(cls, action_name):
        for item in GroundedAction.actionsList:
            if item.name == action_name:
                return item
        return None



class ActionStart:

    actionsList = list()

    def __init__(self, grounded_action, dispatch_time):
        name_act = grounded_action.name.split('#')
        name_act[0] = name_act[0] + '_start'
        self.dispatch_time = dispatch_time
        self.name = '#'.join(name_act) + '$' + str(dispatch_time)
        # Name format: goto_waypoint#robot0#wp0#machine$0

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


    def changeDispatchTime(self, index):
        self.dispatch_time = index
        self.name = self.name.split('$')[0] + '$' + str(index)


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
        print('Dispatch time: ' + str(self.dispatch_time))
        print('Conditions: ' + str(self.conditions))
        print('Effects: ' + str(self.effects))
        print('<<<<<<<<<<<<<<<<<<<<<<<<')


    @classmethod
    def getActionStart(cls, action_name):
        for item in ActionStart.actionsList:
            if item.name == action_name:
                return item
        return None


    @classmethod
    def getActionsWithTime(cls, dispatch_time):
        actions_with_time = list()

        for action in ActionStart.actionsList:
            if action.dispatch_time == dispatch_time:
                actions_with_time.append(action)
        
        return actions_with_time


    def getActionNameWithTime(self, time):
        # Name format: goto_waypoint#robot0#wp0#machine$0
        return self.name.split('$')[0] + '$' + str(time)



class ActionEnd:

    actionsList = list()


    def __init__(self, grounded_action, dispatch_time, start_time):
        name_act = grounded_action.name.split('#')
        name_act[0] = name_act[0] + '_end'
        self.grounded_action = grounded_action
        self.dispatch_time = dispatch_time
        self.start_time = start_time
        self.name = '#'.join(name_act) + '$' + str(dispatch_time)

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


    def changeDispatchTime(self, index):
        self.dispatch_time = index
        self.name = self.name.split('$')[0] + '$' + str(index)


    def changeStartTime(self, index):
        self.start_time = index


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
        print('Dispatch time: ' + str(self.dispatch_time))
        print('Conditions: ' + str(self.conditions))
        print('Effects: ' + str(self.effects))
        print('Over all conditions: ' + str(self.over_all_conditions))
        print('Over all effects: ' + str(self.over_all_effects))
        print('<<<<<<<<<<<<<<<<<<<<<<<<')

    
    def getString(self):
        return self.name


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


    def getConditionPredicates(self):
        predicates_set = set()
        for k, v in self.conditions.items():
            for predicate in v:
                predicates_set.add('#'.join(predicate))
        
        return predicates_set

    @classmethod
    def getActionEnd(cls, action_name):
        for item in ActionEnd.actionsList:
            if item.name == action_name:
                return item
        return None


    @classmethod
    def getActionsWithTime(cls, dispatch_time):
        actions_with_time = list()

        for action in ActionEnd.actionsList:
            if action.dispatch_time == dispatch_time:
                actions_with_time.append(action)
        
        return actions_with_time


    def getActionStart(self):
        name_act = self.grounded_action.name.split('#')
        name_act[0] = name_act[0] + '_start'
        name = '#'.join(name_act) + '$' + str(self.start_time)
        return ActionStart.getActionStart(name)


    def getActionNameWithTime(self, time):
        # Name format: goto_waypoint#robot0#wp0#machine$0
        return self.name.split('$')[0] + '$' + str(time)


''' Creates action instances for all actions in domain file '''
def create_actions(operators):
    for i in range(len(operators)):
        action_name = operators[i].name
        Action(action_name)


def write_predicates_to_file(plans_info):
    file = open('predicate_layers.txt', 'w')
    file.write('PREDICATE LAYERS:\n')
    for plan_info in plans_info:
        file.write('>>>> PLAN ' + i + ' <<<<\n')
        for layer_num in range(len(action_times)):
            file.write('>> Layer: ' + str(layer_num) + '\n')
            for predicate in plan_info['all_predicates']:
                file.write(predicate + '%' + i + '\n')
            file.write('----------------------------\n')
        file.write('/////////////////////////////////\n')
    file.close()


def write_parents_to_file(plans_info):
    file = open('nodes_parents.txt', 'w')
    file.write('NODES AND PARENTS:\n')
    for plan_info in plans_info:
        file.write('>>>> PLAN ' + i + ' <<<<\n')
        for node in plan_info['parents'].keys():
            file.write('>>> Node: ' + node.name + '\n')
            for par in lan_info['parents'][node]:
                file.write(par.name + '\n')
            file.write('----------------------------\n')
        file.write('/////////////////////////////////\n')
    file.close()


def write_actions_to_file(plans_info):
    file = open('actions_par_child.txt', 'w')
    file.write('ACTIONS WITH CHILDREN AND PARENTS:\n')
    for plan_info in plans_info:
        file.write('>>>> PLAN ' + i + ' <<<<\n')
        for action in plan_info['actions'].keys():
            file.write('>>> ACTION: ' + action.name + '\n')
            file.write('> Parents:\n')
            for item in plan_info['actions'][action]['parents']:
                file.write(item.name + '\n')
            file.write('> Children:\n')
            for item in plan_info['actions'][action]['children']:
                file.write(item.name + '\n')
            file.write('----------------------------\n')
        file.write('/////////////////////////////////\n')
    file.close()


def waiting_for_services(operator_service, operator_details_service):
    rospy.wait_for_service(operator_service)
    rospy.wait_for_service(operator_details_service)


def print_plan(plan, action_times):
    print('>>> PRINTING PLAN')
    for i in action_times:
        print('> Time: ' + str(i))
        for action in ActionStart.getActionsWithTime(i):
            print(action.name)
        for action in ActionEnd.getActionsWithTime(i):
            print(action.name)
    print('---------------------------------')


def get_node(all_nodes, name):
        for node in all_nodes:
            if node.name == name:
                return node
        # for node in self.all_nodes:
        #     print('*** ' + node.name)
        # print('>>> ' + name)
        return None


class Graph(object):        ## CLASS TO GENERATE The Dynamic bayes network and to find the probability of success of an plan.
    

    def __init__(self):         ####constructor  method initialize the object
        # self.plans = list of dictionaries
        # Each plans_info has 'plans', which is a list of the plan's action nodes
        # and 'times' which is a list of the times at which actions occurr
        self.received=False
        self.plans_info = list()

    ''' TODO: Why is this callback being executed 2 times? '''
    def total_plan(self, data): ##### call back for the the total order plans from oscar.
        # If condition keeps the callback from running 2 times
        if not self.received:
            # data.esterel_plans is a list of all the totally-ordered plans

            for ordered_plan in data.esterel_plans:

                total_plan = ordered_plan.nodes

                dispatch_times = set()
                plan = list()
                grounded_actions_set = set()
                
                for item in total_plan:
                    name = item.action.name
                    action = Action.getAction(name)
                    
                    param_list = list()
                    for param in item.action.parameters:
                        param_list.append(param.value)
                    
                    
                    # Gets grounded action if it already exists
                    grounded_action = GroundedAction.getGroundedAction(name)
                    # If grounded action does not exist then it creates a new one
                    if grounded_action == None:
                        grounded_action = GroundedAction(action, param_list)

                    grounded_actions_set.add(grounded_action)
                    
                    if item.name[-5:] == 'start':
                        time = round(item.action.dispatch_time, 1)
                        dispatch_times.add(time)
                        node = ActionStart(grounded_action, time)
                    elif item.name[-3:] == 'end':
                        start_time = round(item.action.dispatch_time, 1)
                        time = round(item.action.duration + start_time, 1)
                        dispatch_times.add(time)
                        node = ActionEnd(grounded_action, time, start_time)
                    
                    plan.append(node)
                
                plan_info = dict()
                plan_info['plan'] = plan
                times = list(dispatch_times)
                times.sort()
                plan_info['times'] = times
                plan_info['grounded_actions'] = grounded_actions_set
                plan_info['model'] = BayesianNetwork()
                plan_info['all_nodes'] = set()
                plan_info['parents'] = dict()
                plan_info['children'] = dict()
                plan_info['actions'] = dict()
                plan_info['all_predicates'] = set()

                self.plans_info.append(plan_info)

            self.received=True


    def call_service(self):

        print ("Waiting for service")
        rospy.wait_for_service('/rosplan_knowledge_base/domain/operators')
        rospy.wait_for_service('/rosplan_knowledge_base/domain/operator_details')
        
        print ("Calling Service")
        domain_operators = rospy.ServiceProxy('/rosplan_knowledge_base/domain/operators', GetDomainOperatorService)

        operators = domain_operators().operators
        create_actions(operators)

        #### Subscribing to the node to get the total order plans ######################
        rospy.Subscriber("/csp_exec_generator/valid_plans", EsterelPlanArray, self.total_plan)

        print("waiting for the message")
        while self.received is False:
            continue

        # for loop over each plan
        for plan_info in self.plans_info:
            plan = plan_info['plan']
            dispatch_times = plan_info['times']

            cpd = DiscreteDistribution({'T': 1, 'F': 0})
            
            plan_info['all_predicates'] = set()
            for grounded_action in plan_info['grounded_actions']:
                # Adds the predicates in the set grounded_action.getPredicates() to predicates_set
                plan_info['all_predicates'] |= grounded_action.getPredicates()
            predicates_set = plan_info['all_predicates']
            
            # Adding first layer of predicates
            for predicate in predicates_set:
                node = Node(cpd, name=predicate + '%0')
                plan_info['all_nodes'].add(node)
                plan_info['model'].add_node(node)

            layer_number = 1
            # Cycle which creates the entire Bayes Network until the end
            for time in dispatch_times:
                for action in plan:
                    if action.dispatch_time == time:
                        # Add predicate layer
                        for predicate in predicates_set:
                            node = Node(cpd, name = predicate + '%' + str(layer_number))
                            plan_info['all_nodes'].add(node)
                            plan_info['model'].add_node(node)
                            plan_info['parents'][node] = set()
                            plan_info['parents'][node].add(get_node(plan_info['all_nodes'], predicate + '%' + str(layer_number-1)))
                            plan_info['model'].add_edge(get_node(plan_info['all_nodes'], predicate + '%' + str(layer_number-1)), node)
            
                        # Add action
                        action_node = Node(cpd, name = action.getActionNameWithTime(layer_number))
                        plan_info['model'].add_node(action_node)
                        plan_info['all_nodes'].add(action_node)

                        plan_info['parents'][action_node] = set()
                        plan_info['actions'][action_node] = dict()
                        plan_info['actions'][action_node]['parents'] = set()
                        plan_info['actions'][action_node]['children'] = set()
                        # Connecting to condition predicates
                        for predicate in action.getConditionPredicates():
                            plan_info['model'].add_edge(get_node(plan_info['all_nodes'], predicate + '%' + str(layer_number-1)), action_node)
                            plan_info['parents'][action_node].add(get_node(plan_info['all_nodes'], predicate + '%' + str(layer_number-1)))
                            plan_info['actions'][action_node]['parents'].add(get_node(plan_info['all_nodes'], predicate + '%' + str(layer_number-1)))
                        # Connecting to effects predicates
                        for predicate in action.getEffectsPredicates():
                            plan_info['model'].add_edge(action_node, get_node(plan_info['all_nodes'], predicate + '%' + str(layer_number)))
                            plan_info['parents'][get_node(plan_info['all_nodes'], predicate + '%' + str(layer_number))].add(action_node)
                            plan_info['actions'][action_node]['children'].add(get_node(plan_info['all_nodes'], predicate + '%' + str(layer_number)))

                        if isinstance(action, ActionEnd):
                            # Connecting to action start
                            end_index = layer_number
                            action_start = action.getActionStart()
                            plan_info['model'].add_edge(get_node(plan_info['all_nodes'], action_start.name), action_node)
                            plan_info['parents'][action_node].add(get_node(plan_info['all_nodes'], action_start.name))
                            plan_info['actions'][action_node]['parents'].add(get_node(plan_info['all_nodes'], action_start.name))
                            plan_info['actions'][get_node(plan_info['all_nodes'], action_start.name)]['children'].add(action_node)
                            for j in range(len(dispatch_times)):
                                if dispatch_times[j] == action.start_time:
                                    start_index = j
                            # Connecting to over all predicates
                            for predicate in action.getOverAllPredicates():
                                # Had to add one because the last index is exclusive
                                for j in range(start_index, end_index+1):
                                    plan_info['model'].add_edge(get_node(plan_info['all_nodes'], predicate + '%' + str(j)), action_node)
                                    plan_info['parents'][action_node].add(get_node(plan_info['all_nodes'], predicate + '%' + str(j)))
                                    plan_info['actions'][action_node]['parents'].add(get_node(plan_info['all_nodes'], predicate + '%' + str(j)))
                        
                        layer_number = layer_number + 1

                plan_info['model'].bake()

        write_predicates_to_file(plans_info)
        write_parents_to_file(plans_info)
        write_actions_to_file(plans_info)

        # print_plan(self.plan, self.action_times)

        print('Files successfuly wroten')

        # model.plot()
        # plt.show()

        rospy.spin()



if __name__ == "__main__":

    rospy.init_node('query_client_node', anonymous=True)
    Graph().call_service()
   
    # sys.exit(1)
