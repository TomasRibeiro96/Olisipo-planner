#!/usr/bin/env python
import rospkg
import rospy
import sys
import random

from std_srvs.srv import Empty, EmptyResponse, Trigger, TriggerResponse
from rosplan_dispatch_msgs.srv import DispatchService, DispatchServiceResponse
from rosplan_dispatch_msgs.srv import ExecAlternatives, ExecAlternativesResponse

from rosplan_knowledge_msgs.srv import KnowledgeUpdateServiceArray, KnowledgeUpdateServiceArrayResponse, KnowledgeUpdateServiceArrayRequest, KnowledgeQueryService, KnowledgeQueryServiceResponse, KnowledgeQueryServiceRequest, GetAttributeService, GetAttributeServiceRequest, GetAttributeServiceResponse
from rosplan_knowledge_msgs.msg import KnowledgeItem
from diagnostic_msgs.msg import KeyValue

import os

number_machines_ = 3

pred_probabilities_map_ = dict()


def getProbabilities():
    global pred_probabilities_map_
    
    file = open('/home/tomas/ros_ws/src/ROSPlan/src/rosplan/probabilities-factory_robot.txt', 'r')
    file = open('/home/tomas/ros_ws/src/ROSPlan/src/rosplan/probabilities-skip_door.txt', 'r')
    line = file.readline()
    predicates = True
    while line:
        if line == '-\n':
            predicates = False
        elif predicates:
            split = line.split(' ')
            predicate = split[0]
            spont_false_true = float(split[1])
            spont_true_false = float(split[2].strip('\n'))
            pred_probabilities_map_[predicate] = [spont_false_true, spont_true_false]
        line = file.readline()
    file.close()


def getPredicateKnowledgeItem(predicate):
    attribute_name = predicate.split('#')[0]
    values = predicate.split('#')[1:]

    prop_serv = rospy.ServiceProxy('/rosplan_knowledge_base/state/propositions', GetAttributeService)
    prop_req = GetAttributeServiceRequest()
    prop_req.predicate_name = attribute_name
    prop_resp = prop_serv(prop_req)
    # if predicate == 'door_is_open#door7':
    #     rospy.loginfo('----------------------')
    #     rospy.loginfo('Predicate: ' + predicate)
    #     rospy.loginfo('Knowledge item: \n' + str(prop_resp))
    ki = prop_resp.attributes[0]

    for i in range(len(ki.values)):
        ki.values[i].value = values[i]
    
    # rospy.loginfo('Knowledge item:\n' + str(ki))
    return ki


def perturb():
    try:

        rospy.loginfo('ISR: (/coordinator_adaptable) Starting to perturb')
        # rospy.loginfo('Predicates: ' + str(pred_probabilities_map_.keys()))
        for predicate in pred_probabilities_map_.keys():
            spont_false_true = pred_probabilities_map_[predicate][0]
            spont_true_false = pred_probabilities_map_[predicate][1]

            if predicate == 'door_is_open#door1':
                spont_true_false = 1
            
            if predicate == 'door_is_open#door3':
                spont_false_true = 1

            update_serv = rospy.ServiceProxy('/rosplan_knowledge_base/update_array', KnowledgeUpdateServiceArray)
            update_req = KnowledgeUpdateServiceArrayRequest()

            queryKB = rospy.ServiceProxy('/rosplan_knowledge_base/query_state', KnowledgeQueryService)
            query_KB_req = KnowledgeQueryServiceRequest()

            ki = getPredicateKnowledgeItem(predicate)
            query_KB_req.knowledge.append(ki)

            query_KB_resp = queryKB(query_KB_req)
            is_fact_in_KB = query_KB_resp.results[0]

            number = random.random()
            # rospy.loginfo('------------------------')
            # rospy.loginfo('Predicate: ' + predicate)
            # if is_fact_in_KB:
            #     rospy.loginfo('Fact in KB')
            #     rospy.loginfo('Probability satisfied: ' + str(number < spont_true_false))
            # else:
            #     rospy.loginfo('Fact NOT in KB')
            #     rospy.loginfo('Probability satisfied: ' + str(number < spont_false_true))

            # If fact is not in KB (is False) then add it to
            # the KB with a probability of spont_false_true
            if not is_fact_in_KB and number < spont_false_true:
                # rospy.loginfo('ISR: (/coordinator_adaptable) Adding ' + predicate + ' to KB')
                ki.is_negative = False
                update_req.knowledge.append(ki)
                # update_type.append(0)
                update_req.update_type = [0]
                update_successful = update_serv(update_req)


            # If fact is in KB (is True) then remove it from
            # the KB with a probability of spont_true_false
            elif is_fact_in_KB and number < spont_true_false:
                # rospy.loginfo('ISR: (/coordinator_adaptable) Removing ' + predicate + ' from KB')
                ###############################
                ki.is_negative = True
                update_req.update_type = [0]
                ###############################
                update_req.knowledge.append(ki)
                # update_req.update_type = [2]
                update_successful = update_serv(update_req)

    except rospy.ServiceException, e:
        print 'Unexpected perturbances not called'


def run():
    rospy.init_node('coordinator', anonymous=False)

    # use or not adaptable plan dispatcher
    adaptable_plan_dispatcher_required = rospy.get_param('~adaptable_plan_dispatcher_required', True)
    if adaptable_plan_dispatcher_required:
        print 'using adaptable plan dispatcher'
    else:
        print 'NOT using adaptable plan dispatcher'

    # get execution type from param server
    execution_type = rospy.get_param('~execution_type', 'exec_type_not_set_')

    free_or_non_free = None
    if execution_type == 'iros_problems_free':
        free_or_non_free = 'free_'
    elif execution_type == 'iros_problems_deadlines':
        free_or_non_free = 'non_free_'

    # get execution type from param server
    problem_name = rospy.get_param('~problem_name', 'problem_name_not_set_')

    # for logging purposes, write results of the experiment to a file
    #ros_tcp_port = os.environ['ROS_MASTER_URI'].replace('http://localhost:', '')
    #if adaptable_plan_dispatcher_required:
        #log_file = open('exp_adaptable_' + free_or_non_free + problem_name + '_' + ros_tcp_port + '_n' + '.csv','w')
    #else:
        #log_file = open('exp_non_adaptable_' + free_or_non_free + problem_name + '_' + ros_tcp_port + '_n' + '.csv','w')
    #log_file.write('succeeded?, number of replans, number of executed actions\n')

    goal_achieved = False
    replans = 0

    getProbabilities()

    while not goal_achieved and replans<25:
        rospy.wait_for_service('/rosplan_problem_interface/problem_generation_server')
        rospy.wait_for_service('/rosplan_planner_interface/planning_server')
        rospy.wait_for_service('/rosplan_parsing_interface/parse_plan')
        rospy.wait_for_service('/rosplan_plan_dispatcher/dispatch_plan')
        rospy.wait_for_service('/rosplan_knowledge_base/update_array')
        rospy.wait_for_service('/rosplan_knowledge_base/query_state')
        rospy.wait_for_service('rosplan_knowledge_base/state/propositions')

        try:
            pg = rospy.ServiceProxy('/rosplan_problem_interface/problem_generation_server', Empty)
            pg()

            pi = rospy.ServiceProxy('/rosplan_planner_interface/planning_server', Empty)
            pi()

            pp = rospy.ServiceProxy('/rosplan_parsing_interface/parse_plan', Empty)
            pp()

            if(adaptable_plan_dispatcher_required):
                ea = rospy.ServiceProxy('/csp_exec_generator/gen_exec_alternatives', ExecAlternatives)
                ear = ea()

            perturb()

            dp = rospy.ServiceProxy('/rosplan_plan_dispatcher/dispatch_plan', DispatchService)
            dsr = dp()

            goal_achieved = dsr.goal_achieved
            if not dsr.goal_achieved:
                replans += 1

        except rospy.ServiceException, e:
            replans += 1

    # get number of executed actions
    number_of_executed_actions = 0
    try:
        ac = rospy.ServiceProxy('/action_count', Trigger)
        acr = ac()
        print 'Actions: ' + acr.message
        number_of_executed_actions = acr.message
    except rospy.ServiceException, e:
        print 0

    # check if goal was achieved, write to log file
    if goal_achieved:
        print 'SUCCESS ', str(replans)
        #log_file.write('true, ' + str(replans) + ', ' + str(number_of_executed_actions) + '\n')
    else:
        print 'FAILED ', str(replans)
        #log_file.write('false' + str(replans) + ', ' + str(number_of_executed_actions) + '\n')

    # for logging purposes, write experiment results to text file, closing the file since we are done
    #log_file.close()


if __name__ == '__main__':
    run()
