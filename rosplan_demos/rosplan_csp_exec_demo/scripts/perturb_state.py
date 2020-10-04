#!/usr/bin/env python
import rospkg
import rospy
import sys
import random

from rosplan_knowledge_msgs.srv import KnowledgeUpdateServiceArray, KnowledgeUpdateServiceArrayResponse, KnowledgeUpdateServiceArrayRequest, KnowledgeQueryService, KnowledgeQueryServiceResponse, KnowledgeQueryServiceRequest, GetDomainAttributeService, GetDomainAttributeServiceRequest, GetDomainAttributeServiceResponse, PerturbStateService, PerturbStateServiceResponse, GetAttributeService, GetAttributeServiceRequest, GetAttributeServiceResponse
from rosplan_knowledge_msgs.msg import KnowledgeItem
from diagnostic_msgs.msg import KeyValue

import os

pred_probabilities_map_ = dict()

predicates_list_ = list()

current_state_ = list()

def getProbabilities():
    global pred_probabilities_map_
    
    prob_file_path = rospy.get_param('~probabilities_file')
    file = open(prob_file_path, 'r')
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


def getListOfAllPredicates():
    global predicates_list_

    prop_serv = rospy.ServiceProxy('/rosplan_knowledge_base/domain/predicates', GetDomainAttributeService)
    prop_req = GetDomainAttributeServiceRequest()
    predicates_list_ = prop_serv(prop_req).items

    # rospy.loginfo('ISR: (%s) Predicates list:\n%s', rospy.get_name(), str(predicates_list_))


def getPredicateKnowledgeItem(predicate):
    predicate_name = predicate.split('#')[0]
    values = predicate.split('#')[1:]

    # rospy.loginfo('ISR: (/perturb_server) Predicate: ' + str(predicate))
    # rospy.loginfo('ISR: (/perturb_server) Predicate name: ' + str(predicate_name))
    # rospy.loginfo('ISR: (/perturb_server) Values: ' + str(values))

    ki = KnowledgeItem()
    ki.knowledge_type = ki.FACT
    ki.is_negative = False
    ki.attribute_name = predicate_name
    ki_values = list()

    i = 0

    for pred in predicates_list_:
        if pred.name == predicate_name:
            for kv in pred.typed_parameters:
                key_value = KeyValue()
                key_value.key = kv.key
                key_value.value = values[i]
                ki_values.append(key_value)
                i = i + 1
            break

    ki.values = ki_values

    return ki


def convertKnowledgeItemtoString(ki):
    name = ki.attribute_name
    for item in ki.values:
        name = name + '#' + item.value
    return name


def fillCurrentState():
    global current_state_
    current_state_ = list()

    get_state_srv = rospy.ServiceProxy('/rosplan_knowledge_base/state/propositions', GetAttributeService)
    get_state_req = GetAttributeServiceRequest()
    get_state_req.predicate_name = ''
    state = get_state_srv(get_state_req).attributes

    for ki in state:
        current_state_.append(convertKnowledgeItemtoString(ki))


def getListOfMachinesWorkingAndMaintained():
    machines_maintained = list()
    machines_working = list()
    for predicate in current_state_:
        if predicate.split('#')[0] == 'machine_is_maintained':
            machines_maintained.append(predicate.split('#')[1])
        else:
            machines_working.append(predicate.split('#')[1])

    return machines_working, machines_maintained


def perturb(req):

    # rospy.loginfo('ISR: (%s) Starting to perturb', rospy.get_name())
    # rospy.loginfo('Predicates: ' + str(pred_probabilities_map_.keys()))

    fillCurrentState()

    # Service to add or remove fact to knowledge base
    update_serv = rospy.ServiceProxy('/rosplan_knowledge_base/update_array', KnowledgeUpdateServiceArray)
    update_req = KnowledgeUpdateServiceArrayRequest()

    machines_working, machines_maintained = getListOfMachinesWorkingAndMaintained()

    for predicate in current_state_:
        if predicate.split('#')[0] == 'machine_is_maintained':
            machines_maintained.append(predicate.split('#')[1])
        else:
            machines_working.append(predicate.split('#')[1])
    
    # If machine is maintained then set it to false with respective probability
    for machine in machines_maintained:
        predicate_name = 'machine_is_maintained#'+machine
        spont_true_false = pred_probabilities_map_[predicate_name][1]

        if random.random() < spont_true_false:
            # rospy.loginfo('ISR: (%s) Removing %s from KB', rospy.get_name(), predicate_name)
            ki = getPredicateKnowledgeItem(predicate_name)
            update_req.update_type = [2]
            update_req.knowledge.append(ki)
            update_successful = update_serv(update_req)
    

    for machine in machines_working:
        if machine not in machines_maintained:
            predicate_name = 'machine_is_working#'+machine
            spont_true_false_pred = pred_probabilities_map_[predicate_name][1]

            maintain_predicate_name = 'machine_is_maintained#'+machine
            maintain_spont_false_true = pred_probabilities_map_[maintain_predicate_name][0]

            # If machine is not maintained and it's working, then set
            # it to false with a probability of spont_true_false_pred
            if random.random() < spont_true_false_pred:
                # rospy.loginfo('ISR: (%s) Removing %s from KB', rospy.get_name(), predicate_name)
                ki = getPredicateKnowledgeItem(predicate_name)
                update_req.update_type = [2]
                update_req.knowledge.append(ki)
                update_successful = update_serv(update_req)
            # If machine remains working then set it to maintained
            # with a probability of maintain_spont_false_true
            elif random.random() < maintain_spont_false_true:
                # rospy.loginfo('ISR: (%s) Adding %s from KB', rospy.get_name(), maintain_predicate_name)
                ki = getPredicateKnowledgeItem(maintain_predicate_name)
                ki.is_negative = False
                update_req.knowledge.append(ki)
                update_req.update_type = [0]
                update_successful = update_serv(update_req)


    return PerturbStateServiceResponse(True)


if __name__ == '__main__':
    rospy.init_node('perturb_state_server')

    # rospy.loginfo('ISR: (/perturb_server) Waiting for services')
    rospy.wait_for_service('/rosplan_knowledge_base/domain/predicates')
    rospy.wait_for_service('/rosplan_knowledge_base/update_array')
    rospy.wait_for_service('/rosplan_knowledge_base/query_state')
    rospy.wait_for_service('/rosplan_knowledge_base/state/propositions')

    getProbabilities()
    getListOfAllPredicates()

    rospy.Service('perturb_state', PerturbStateService, perturb)

    # rospy.loginfo('ISR: (%s) Spinning', , rospy.get_name())

    rospy.spin()
