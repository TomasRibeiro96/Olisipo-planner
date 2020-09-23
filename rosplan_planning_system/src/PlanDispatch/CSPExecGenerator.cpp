/*
 * Copyright [2019] <KCL - ISR collaboration>
 *
 * KCL: King's College London
 * ISR: Institue for Systems and Robotics
 *
 * Author: Michael Cashmore (michael.cashmore@kcl.ac.uk), Oscar Lima (olima@isr.tecnico.ulisboa.pt)
 *
 * Finds out many different alternatives for a esterel plan to be executed.
 *
 */

#include <rosplan_planning_system/PlanDispatch/CSPExecGenerator.h>
#include <algorithm>

int total_number_nodes_expanded = 1;

// TODO: When service is called again, if the at_start effects of an action
// are alreasdy present in the current state, then remove it from the open list

CSPExecGenerator::CSPExecGenerator() : nh_("~"), is_esterel_plan_received_(false), max_search_depth_(0)
{
    // subscriptions: subscribe to esterel plan, a fully ordered plan
    sub_esterel_plan_ = nh_.subscribe("/rosplan_parsing_interface/complete_plan", 1, &CSPExecGenerator::esterelPlanCB, this);

    // publications, executions alternatives (multiple esterel plans)
    pub_valid_plans_ = nh_.advertise<rosplan_dispatch_msgs::EsterelPlanArray>("valid_plans", 1);

    // remove, publish test esterel plan
    pub_esterel_plan_ = nh_.advertise<rosplan_dispatch_msgs::EsterelPlan>("partial_order_plan", 1);

    // services: compute different execution alternatives from a partially ordered esterel plan (a plan
    // with no conditional edges, but only interference edges)
    srv_gen_alternatives_ = nh_.advertiseService("gen_exec_alternatives", &CSPExecGenerator::srvCB, this);
    
    ros::NodeHandle nh;
    calculate_actions_prob_client_ = nh.serviceClient<rosplan_dispatch_msgs::CalculateActionsProbabilityService>("getActionsJointProbability");

    calculate_full_prob_client_ = nh.serviceClient<rosplan_dispatch_msgs::CalculateFullProbabilityService>("calculateFullJointProbability");

    get_nodes_layers_client_ = nh.serviceClient<rosplan_dispatch_msgs::GetNodesLayersService>("getNodesLayers");

    setup_client_ = nh.serviceClient<rosplan_dispatch_msgs::SetupService>("setup");

    backtrack_client_ = nh.serviceClient<rosplan_dispatch_msgs::BacktrackService>("backtrack");

    // mirror KB (query real KB and get its data) but without facts and goals
    action_simulator_.init();

    // to cap max search depth to a certain value, get from param server
    nh_.param<int>("max_search_depth", max_search_depth_, 100);
}

CSPExecGenerator::~CSPExecGenerator()
{
    // shut down publishers and subscribers
    sub_esterel_plan_.shutdown();
}

void CSPExecGenerator::printNodes(std::string msg, std::vector<int> &nodes)
{
    std::stringstream ss;
    for(auto nit=nodes.begin(); nit!=nodes.end(); nit++) {
        std::string action_name;
        std::vector<std::string> params;
        bool action_start;
        int action_id;
        if(!getAction(*nit, action_name, params, original_plan_, action_start, action_id)) {
            ROS_ERROR("failed to get action properties (while applying action)");
        }
        std::string full_name = buildActionName(action_name, params, action_start);
        ss << full_name;
        ss << " | ";
    }
    ROS_INFO("%s: {%s}", msg.c_str(), ss.str().c_str());
}

void CSPExecGenerator::esterelPlanCB(const rosplan_dispatch_msgs::EsterelPlan::ConstPtr& msg)
{
    ROS_INFO("esterel plan received");
    original_plan_ = *msg;

    // raise flag to indicate a msg has been received in callback
    is_esterel_plan_received_ = true;
}

void CSPExecGenerator::initConstraints(std::map<int, int> &set_of_constraints)
{
    // construct set of constraints from original received esterel plan
    // example:
    // C : [2:3, 5:6]; node 3 goes after node 2, node 6 goes after node 5

    // delete previous data if any
    set_of_constraints.clear();

    // to print constraints at the end
    std::stringstream ss;

    // init set of temporal constraints (C)
    for(auto eit=original_plan_.edges.begin(); eit!=original_plan_.edges.end(); eit++) {
        // discriminate for interference edges
        if(eit->edge_type == rosplan_dispatch_msgs::EsterelPlanEdge::INTERFERENCE_EDGE ||
            eit->edge_type == rosplan_dispatch_msgs::EsterelPlanEdge::START_END_ACTION_EDGE) {
            // add constraint
            set_of_constraints.insert(std::pair<int, int>(eit->source_ids[0], eit->sink_ids[0]));

            // add constraint to print buffer
            ss << "(" << eit->source_ids[0] << ":" << eit->sink_ids[0] << ")";
            continue;
        }
        // NOTE: condition edges are ignored: we consider only a partially orderes plan
    }

    // print constraints
    ROS_DEBUG("constraints: %s", ss.str().c_str());
}

bool CSPExecGenerator::checkTemporalConstraints(std::vector<int> &set_of_ordered_nodes,
    std::map<int, int> &set_of_constraints)
{
    // check if set of ordered nodes (F) satisfies the set of constraints (C)

    // iterate over set of constraints
    for(auto cit=set_of_constraints.begin(); cit!=set_of_constraints.end(); cit++) {
        // find key in set_of_ordered_nodes
        std::vector<int>::iterator kit = std::find(set_of_ordered_nodes.begin(),
                            set_of_ordered_nodes.end(), cit->first);
        // check if element was found
        if(kit != set_of_ordered_nodes.end()) {
            // key was found, now find value
            std::vector<int>::iterator vit = std::find(set_of_ordered_nodes.begin(),
                            set_of_ordered_nodes.end(), set_of_constraints.find(cit->first)->second);
            // check if value was found
            if(vit != set_of_ordered_nodes.end())
                // if vit index < kit index, then constraint is violated
                if(std::distance(kit, vit) < 0)
                    return false;
        }
    }

    return true;
}

void CSPExecGenerator::testFunctions()
{
    // set here what you want to test
    bool test_temporal_constraints = false;

    if(test_temporal_constraints) {
        // example of how to use CheckTemporalConstraints()

        ROS_INFO("testing temporal constraints now");
        std::vector<int> set_of_ordered_nodes = {1, 3, 2, 4, 5, 6}; // violates constraints
        // std::vector<int> set_of_ordered_nodes = {1, 2, 3, 4, 5, 6}; // does not violate constraints
        std::map<int, int> set_of_constraints = {{2, 3},{5, 6}};
        if(checkTemporalConstraints(set_of_ordered_nodes, set_of_constraints))
            ROS_DEBUG("constraints are satisfied");
        else
            ROS_DEBUG("constraints are violated");

        // test function findNodesBeforeA
        std::vector<int> open_list = {1, 4, 3, 2};
        std::vector<int> s = findNodesBeforeA(3, open_list);
        //print s
        std::stringstream ss;
        for(auto nit=s.begin(); nit!=s.end(); nit++) {
            ss << *nit;
            ss << ",";
        }
        ROS_DEBUG("nodes before a : {%s}", ss.str().c_str());
    }
}

bool CSPExecGenerator::getAction(int node_id, std::string &action_name, std::vector<std::string> &params,
    rosplan_dispatch_msgs::EsterelPlan &plan, bool &action_start, int &action_id)
{
    // input a node id and return the action name and params

    // delete previous data if any
    params.clear();

    // iterate over the original plan
    for(auto nit=plan.nodes.begin(); nit!=plan.nodes.end(); nit++) {

        // check if node id matches with node
        if(nit->node_id == node_id) {

            // get name, write return value (1) by reference
            action_name = nit->action.name;
            action_id = nit->action.action_id;

            // extract params
            for(auto pit=nit->action.parameters.begin(); pit!=nit->action.parameters.end(); pit++)
                // write return value (2) by reference
                params.push_back(pit->value);

            // discriminate node action start or end
            if(nit->node_type == rosplan_dispatch_msgs::EsterelPlanNode::ACTION_START) {
                // write return value (3) by reference
                action_start = true;
                return true;
            }
            else if(nit->node_type == rosplan_dispatch_msgs::EsterelPlanNode::ACTION_END) {
                // write return value (3) by reference
                action_start = false;
                return true;
            }
            else {
                ROS_ERROR("node should be either start or end, while getting action (id: %d)", node_id);
                return false;
            }
        }
    }

    ROS_ERROR("get action: node id : %d, was not found in plan", node_id);
    return false;
}

bool CSPExecGenerator::getStartNodeID(int end_node_id, int &action_start_node_id)
{
    // receive as input a node id from a end action node,
    // return by reference the node id of the correspondent action start

    // get the node from the node id
    for(auto nit=original_plan_.nodes.begin(); nit!=original_plan_.nodes.end(); nit++) {
        // find needed node by comparing id
        if(!(nit->node_id == end_node_id))
            continue;

        // id matches, now ensure it is an action end node
        if(!(nit->node_type == rosplan_dispatch_msgs::EsterelPlanNode::ACTION_END)) {
            ROS_ERROR("Cannot return action start node id, Node id matches but is not an action end");
            return false;
        }

        // node is an action end node, find its correspondent start node id

        // iterate over the edges in id's
        for(auto enit=nit->edges_in.begin(); enit!=nit->edges_in.end(); enit++) {
            // get complete edge msg from edge id
            for(auto eit=original_plan_.edges.begin(); eit!=original_plan_.edges.end(); eit++) {
                // ignore edges which id don't match
                if(eit->edge_id != *enit) {
                    continue;
                }

                // ensure is an "action end" edge
                if(eit->edge_type==rosplan_dispatch_msgs::EsterelPlanEdge::START_END_ACTION_EDGE) {
                    // return by reference the node id of the start action
                    action_start_node_id = eit->source_ids[0];
                    ROS_DEBUG("found correspondent action start node (%d) from action end node (%d)", action_start_node_id, end_node_id);
                    return true;
                }
            }
        }
    }

    ROS_ERROR("failed to get action start node id (of action end :%d)", end_node_id);
    return false;
}

bool CSPExecGenerator::validNodes(std::vector<int> &open_list, std::vector<int> &valid_nodes)
{
    // iterate over open list (O), check if node preconditions are met in current state (S)

    // ensure open list is not empty
    if(!open_list.size() > 0) {
        ROS_DEBUG("open list is empty, while checking validNodes");
        return false;
    }

    // iterate over open list
    for(auto nit=open_list.begin(); nit!=open_list.end(); nit++) {

        // get action properties (name, params, type) from node id
        std::string action_name;
        std::vector<std::string> params;
        bool action_start;
        int action_id;
        if(!getAction(*nit, action_name, params, original_plan_, action_start, action_id)) {
            ROS_ERROR("failed to get action properties (while getting valid nodes)");
            return false;
        }

        if(action_start) {
            // check if action start + overall preconditions are met
            ROS_DEBUG("check if action start (id: %d) is applicable : (%s)", *nit,
                         action_simulator_.convertPredToString(action_name, params).c_str());

            double action_probability; // value will get written here by reference
            if(action_simulator_.isActionStartApplicable(action_name, params, action_probability)) {
                ROS_DEBUG("(action start) node is valid (id: %d), add to valid list", *nit);
                // node is valid, add to list
                valid_nodes.push_back(*nit);

                // store (or update) action probability in map
                action_prob_map_[*nit] = action_probability;
            }
            else
                ROS_DEBUG("(action start) node %d is NOT valid", *nit);
        }
        else {
            // check if action end + overall preconditions are met
            ROS_DEBUG("check if action end (id: %d) is applicable : (%s)", *nit,
                         action_simulator_.convertPredToString(action_name, params).c_str());

            double action_probability; // value will get written here by reference
            if(action_simulator_.isActionEndApplicable(action_name, params, action_probability)) {
                ROS_DEBUG("(action end) node is valid, check if correspondent action start is ordered");

                // Ignore action ends in validNodes for actions that have not started
                // get action id of start node
                int start_node_id;
                if(!getStartNodeID(*nit, start_node_id))
                    return false;

                // add only if start node is already ordered
                bool ordered = false;
                for(auto onit=ordered_nodes_.begin(); onit!=ordered_nodes_.end(); onit++) {
                    if(start_node_id == *onit) {
                        ordered = true;
                        // node is valid, add to list
                        valid_nodes.push_back(*nit);
                        ROS_DEBUG("checked if correspondent action start is ordered : yes is ordered, add action (%d) to valid list", *nit);
                    }
                }

                if(!ordered) {
                    for(auto eait=action_executing_.begin(); eait!=action_executing_.end(); eait++) {
                        if((*nit-1) == *eait) {
                            ordered = true;
                            // node is valid, add to list
                            valid_nodes.push_back(*nit);
                            ROS_DEBUG("checked if correspondent action start is ordered : it is already executing, add action (%d) to valid list", *nit);
                        }
                    }
                }

                if(!ordered) {
                    ROS_DEBUG("skipping applicable action end (%d) because action start (%d) is not ordered yet", *nit, start_node_id);
                }
                else {
                    // store (or update) action probability in map
                    action_prob_map_[*nit] = action_probability;
                    // action_prob_map_.insert(std::pair<int, double>(*nit, action_probability));
                }
                // printNodes("ordered nodes F", ordered_nodes_);
            }
        }
    }

    // print valid nodes
    std::stringstream ss;
    if(valid_nodes.size() > 0) {
        //printNodes("valid nodes", valid_nodes);
        return true;
    }
    else
        ROS_DEBUG("no valid nodes were found");

    // no valid nodes
    return false;
}

std::vector<int> CSPExecGenerator::findNodesBeforeA(int a, std::vector<int> &open_list)
{
    // find all nodes b in open list (O) which ordering constraints enforce them before a

    std::vector<int> nodes_before_a;

    // make sure constraints are non empty
    if(set_of_constraints_.size() > 0) {
        // iterate over the constraints
        for(auto cit=set_of_constraints_.begin(); cit!=set_of_constraints_.end(); cit++) {
            if(cit->second == a) {
                // relevant constraint, find correspondent node in open list
                for(auto oit=open_list.begin(); oit!=open_list.end() ;oit++) {
                    if(*oit == cit->first)
                        // found constraint in open list which should happen before a, aka skipped node
                        // this happens when e.g. a human helps the robot to do the action
                        nodes_before_a.push_back(cit->first);
                }
            }
        }
    }

    return nodes_before_a;
}

void CSPExecGenerator::backtrack(std::string reason_for_backtrack, bool backtrack_bayes_network)
{
    // backtrack: popf, remove last element from f, store in variable and revert that action
    ROS_DEBUG("backtrack because %s", reason_for_backtrack.c_str());
    if(!ordered_nodes_.empty()) { // ensure stack is not empty
        // revert action
        ROS_DEBUG("poping action (removing from stack), reverting action to S, action id: %d", ordered_nodes_.back());
        std::string action_name;
        std::vector<std::string> params;
        bool action_start;
        int action_id;
        if(getAction(ordered_nodes_.back(), action_name, params, original_plan_, action_start, action_id)) {
            ROS_DEBUG("KB after reverting action %d", ordered_nodes_.back());

            ordered_nodes_.pop_back(); // eliminate last node from stack
            // getAction() finds out if last element of "f" is action start or end, info is in action_start boolean
            if(action_start)
                action_simulator_.revertActionStart(action_name, params);
            else
                action_simulator_.revertActionEnd(action_name, params);
            
            if(backtrack_bayes_network){
                // ROS_INFO("||| Backtrack: %s |||", reason_for_backtrack.c_str());
                backtrackBayesianNetwork();
            }
            // action_simulator_.printInternalKBFacts();
        }
        else
            ROS_ERROR("failed to get action properties (while backtracking)");
    }
}

double CSPExecGenerator::computePlanProbability(std::vector<int> &ordered_nodes,
        std::map<int, double> &action_prob_map)
{
    double combined_probability = 1.0;

    //iterate over the plan
    for(auto pit=ordered_nodes_.begin(); pit!=ordered_nodes_.end(); pit++) {
        // get correspondent action probability from map
        std::map<int, double>::const_iterator prob_it = action_prob_map.find(*pit);
        if (prob_it == action_prob_map.end()) {
            //handle the error
            ROS_ERROR("could not found correspondent probability in map");
            return -1.0;
        } else {
            // propagate probabilities along the plan
            combined_probability *= prob_it->second;
        }
    }

    return combined_probability;
}

std::string CSPExecGenerator::getStateAsString(std::vector<rosplan_knowledge_msgs::KnowledgeItem> state){
    std::stringstream ss;
    for(auto&& fact1: state){
        ss << action_simulator_.convertPredToString(fact1);
        ss << ", ";
    }
    return ss.str();
}

bool CSPExecGenerator::factsAreEqual(rosplan_knowledge_msgs::KnowledgeItem fact1, rosplan_knowledge_msgs::KnowledgeItem fact2){
    return action_simulator_.convertPredToString(fact1) == action_simulator_.convertPredToString(fact2);
}

bool CSPExecGenerator::statesAreEqual(std::vector<rosplan_knowledge_msgs::KnowledgeItem> state1,
                                        std::vector<rosplan_knowledge_msgs::KnowledgeItem> state2){
    
    // If states have a different number of facts then they are different
    if(state1.size() != state2.size())
        return false;

    // ROS_INFO(">>> State 1: %s", getStateAsString(state1).c_str());

    // ROS_INFO(">>> State 2: %s", getStateAsString(state2).c_str());

    for(auto&& fact1: state1){
        bool fact_found = false;
        for(auto&& fact2: state2){

            // If fact1 is found in state2, then we can stop searching state2
            // and start searching for the next fact1 in state2
            if(factsAreEqual(fact1, fact2)){
                fact_found = true;
                break;
            }
        }
        // If fact1 was not found in state2 then state1 and state2 are different
        if(!fact_found)
            return false;
    }
    return true;

}

std::string CSPExecGenerator::getFullActionName(int a){
    std::string action_name;
    std::vector<std::string> params;
    bool action_start;
    int action_id;
    if(!getAction(a, action_name, params, original_plan_, action_start, action_id)) {
        ROS_ERROR("failed to get action properties (while getting name");
    }
    return buildActionName(action_name, params, action_start);
}

std::string CSPExecGenerator::buildActionName(std::string action_name, std::vector<std::string> params, bool action_start){
    std::stringstream ss;
    ss << action_name;
    if(action_start){
        ss << "_start";
    }
    else{
        ss << "_end";
    }
    for(auto&& parameter: params) {
        ss << "#";
        ss << parameter;
    }
    return ss.str();
}

bool CSPExecGenerator::simulateAction(bool action_start, std::string action_name, std::vector<std::string> params){
    if(action_start) {
        // action start
        ROS_DEBUG("apply action a : (%s)", action_simulator_.convertPredToString(action_name, params).c_str());
        if(!action_simulator_.simulateActionStart(action_name, params)) {
            ROS_ERROR("could not simulate action start");
            return false;
        }
    }
    else {
        // action end
        if(!action_simulator_.simulateActionEnd(action_name, params)) {
            ROS_ERROR("could not simulate action end");
            return false;
        }
    }
    return true;
}


std::vector<rosplan_knowledge_msgs::KnowledgeItem> CSPExecGenerator::getStateAfterAction(std::string action_name,
                                                                                        std::vector<std::string> params,
                                                                                        std::vector<int> open_list){

    std::string action1 = buildActionName(action_name, params, true);

    for(auto a=open_list.begin(); a!=open_list.end(); a++){
        // get action properties (name, params, type) from node id
        std::string name;
        std::vector<std::string> parameters;
        bool action_start;
        int action_id;
        if(!getAction(*a, name, parameters, original_plan_, action_start, action_id)) {
            ROS_ERROR("failed to get action properties (while applying action)");
        }
        
        std::string action2 = buildActionName(name, parameters, action_start);

        if(action1 == action2){
            // ROS_INFO("Found action");
            action1 = buildActionName(action_name, params, false);
            if(!simulateAction(action_start, name, parameters)){
                ROS_ERROR("Could not simulate action: %s", action2.c_str());
            }
            ordered_nodes_.push_back(*a);
            // We restart the loop because the action end may be
            // before the action start in the open_list
            if(action_start)
                a = open_list.begin();
            else
                break;
            
        }
    }

    std::vector<rosplan_knowledge_msgs::KnowledgeItem> state_after_action = action_simulator_.getCurrentState();

    backtrack("Got state after action 1", false);
    backtrack("Got state after action 2", false);

    return state_after_action;
}

std::vector<std::string> CSPExecGenerator::split(std::string strToSplit, char delimeter){
    // std::string split implementation by using delimiter as a character.
    std::stringstream ss(strToSplit);
    std::string item;
    std::vector<std::string> splittedStrings;
    while (std::getline(ss, item, delimeter))
    {
       splittedStrings.push_back(item);
    }
    return splittedStrings;
}

void CSPExecGenerator::printVectorOfStrings(std::string msg, std::vector<std::string> vec){
    std::stringstream ss;
    ss << msg;
    for(auto&& item: vec){
        ss << item;
        ss << ", ";
    }
    ROS_INFO("%s", ss.str().c_str());
}

void CSPExecGenerator::printExpectedFacts(){
    ROS_INFO(">>> Expected facts:");
    for(int i = 0; i < expected_facts_.size(); i++){
        std::stringstream ss;
        ss << "Layer ";
        ss << i;
        ss << ": ";
        for(auto&& predicate: expected_facts_[i]){
            ss << action_simulator_.convertPredToString(predicate);
            ss << ", ";
        }
        ROS_INFO("%s", ss.str().c_str());
    }
}

void CSPExecGenerator::fillExpectedFacts(std::vector<std::string> expected_predicates){

    expected_facts_.clear();

    // printVectorOfStrings("Expected predicates: ", expected_predicates);

    int index = 0;
    std::vector<rosplan_knowledge_msgs::KnowledgeItem> current_layer;

    for(auto&& predicate: expected_predicates){
        std::vector<std::string> name_index = split(predicate, '%');
        // Get index of fact (layer)
        int node_index = std::stoi(name_index[1]);
        std::vector<std::string> name_params = split(name_index[0], '#');
        // Name of predicate
        std::string predicate_name = name_params[0];
        // Parameters of predicate
        std::vector<std::string> predicate_params = name_params;
        predicate_params.erase(predicate_params.begin());

        rosplan_knowledge_msgs::KnowledgeItem fact = action_simulator_.createFactKnowledgeItem(predicate_name, predicate_params, false);

        if(node_index == index){
            // ROS_INFO("Added to current layer");
            current_layer.push_back(fact);
        }
        else{
            // ROS_INFO("Added to new layer");
            expected_facts_.push_back(current_layer);
            current_layer.clear();
            current_layer.push_back(fact);
            index = node_index;
        }
    }
    expected_facts_.push_back(current_layer);

    // printExpectedFacts();

}

PyObject* CSPExecGenerator::convertVectorToPythonList(std::vector<std::string> plan_with_names){
    
    PyObject* pList = PyList_New(plan_with_names.size());
    PyObject* pName;

    for(int i = 0; i < plan_with_names.size(); i++){
        pName = PyUnicode_FromString(plan_with_names[i].c_str());
        PyList_SetItem(pList, i, pName);
    }

    return pList;
}

PyObject* CSPExecGenerator::convertVectorToPythonTuple(std::vector<std::string> plan_with_names){
    
    PyObject* pTuple = PyTuple_New(plan_with_names.size());
    PyObject* pName;

    for(int i = 0; i < plan_with_names.size(); i++){
        pName = PyUnicode_FromString(plan_with_names[i].c_str());
        PyTuple_SetItem(pTuple, i, pName);
    }

    return pTuple;
}

std::vector<std::string> CSPExecGenerator::convertCPyObjectToVector(CPyObject pList){
    
    if(!pList){
        ROS_ERROR("Expected predicates list is null");
    }

    std::vector<std::string> vec;
    Py_ssize_t size = PyList_Size(pList);
    std::string predicate;

    for(int i = 0; i < size; i++){
        PyObject* item = PyList_GetItem(pList, i);
        predicate = PyBytes_AsString(item);
        vec.push_back(predicate);
    }

    return vec;
}

double CSPExecGenerator::getCurrentPlanProbabilityAndFillExpectedFacts(){

    // double plan_success_probability;
    // std::vector<std::string> expected_predicates;

    // // ROS_INFO("Calling calculateFullJointProbability");
    // CPyObject pFuncCalcProb = PyObject_GetAttrString(pModule_, "calculateFullJointProbability");
    // if(pFuncCalcProb && PyCallable_Check(pFuncCalcProb)){
        
    //     CPyObject pReturnedProbability = PyObject_CallObject(pFuncCalcProb, NULL);
    //     plan_success_probability = PyFloat_AsDouble(pReturnedProbability);

    //     if(plan_success_probability == -1.0){
    //         ROS_ERROR("Received NULL from calculateFullJointProbability");
    //     }
    //     else{
    //         // ROS_INFO("Full joint probability: %f", plan_success_probability);
    //     }

    //     // ROS_INFO("Calling getNodesLayers");
    //     CPyObject pFuncGetLayers = PyObject_GetAttrString(pModule_, "getNodesLayers");
    //     if(pFuncGetLayers && PyCallable_Check(pFuncGetLayers)){
    //         // ROS_INFO("Got answer from getNodesLayers");
    //         CPyObject pReturned = PyObject_CallObject(pFuncGetLayers, NULL);
    //         // ROS_INFO("Converting CPyObject to Vector");
    //         expected_predicates = convertCPyObjectToVector(pReturned);
    //         // ROS_INFO("Filling expected facts");
    //         fillExpectedFacts(expected_predicates);
    //     }
    //     else{
    //         PyErr_Print();
    //         ROS_ERROR("ERROR: Failed to call getNodesLayers");
    //     }
    
    // }
    // else{
    //     PyErr_Print();
    //     ROS_ERROR("ERROR: Unable to import calculateFullJointProbability");
    // }

    ////////////////////////////////////////////////////////////
    rosplan_dispatch_msgs::CalculateFullProbabilityService srv;
    double plan_success_probability;
    if(calculate_full_prob_client_.call(srv)){
        plan_success_probability = srv.response.plan_success_probability;
        // ROS_INFO("ISR: (%s) Received response of full joint: %f", ros::this_node::getName().c_str(), plan_success_probability);
    }
    else{
        // ROS_INFO("ISR: (%s) Did NOT receive response of full joint", ros::this_node::getName().c_str());
    }

    rosplan_dispatch_msgs::GetNodesLayersService srv2;
    if(get_nodes_layers_client_.call(srv2)){
        std::vector<std::string> expected_facts = srv2.response.expected_facts;
        fillExpectedFacts(expected_facts);
        // ROS_INFO("ISR: (%s) Received response of getNodesLayers", ros::this_node::getName().c_str());
    }
    else{
        // ROS_INFO("ISR: (%s) Did NOT receive response of getNodesLayers", ros::this_node::getName().c_str());
    }

    ////////////////////////////////////////////////////////////
    return plan_success_probability;
}

void CSPExecGenerator::backtrackBayesianNetwork(){
    // // ROS_INFO("Calling backtrack");
    // CPyObject pFuncCalcProb = PyObject_GetAttrString(pModule_, "backtrack");
    // if(pFuncCalcProb && PyCallable_Check(pFuncCalcProb)){
        
    //     PyObject_CallObject(pFuncCalcProb, NULL);
    // }

    rosplan_dispatch_msgs::BacktrackService srv;
    if(backtrack_client_.call(srv)){
        if(srv.response.success){
            // ROS_INFO("ISR: (%s) Successfully backtracked bayesian network", ros::this_node::getName().c_str());
        }
        else{
            // ROS_INFO("ISR: (%s) Something went wrong in backtrack method", ros::this_node::getName().c_str());
        }
    }
    else{
        // ROS_INFO("ISR: (%s) Did NOT backtrack bayesian network", ros::this_node::getName().c_str());
    }

}

bool CSPExecGenerator::orderNodes(std::vector<int> open_list, int &number_expanded_nodes)
{
    // shift nodes from open list (O) to ordered plans (R)
    // offering all possible different execution alternatives via DFS (Depth first search)

    ROS_DEBUG("order nodes (recurse)");

    if(!checkTemporalConstraints(ordered_nodes_, set_of_constraints_)) {
        // ROS_INFO("$$$ Temporal constraints not satisfied $$$$");
        backtrack("Temporal constraints not satisfied", true);
        return false;
    }

    // check if goals are achieved
    ROS_DEBUG("checking if goals are achieved...");
    if(action_simulator_.areGoalsAchieved()) {

        // convert list of orderes nodes into esterel plan (reuses the originally received esterel plan)
        rosplan_dispatch_msgs::EsterelPlan esterel_plan_msg = convertListToEsterel(ordered_nodes_);

        // add new valid ordering to ordered plans (R)
        exec_aternatives_msg_.esterel_plans.push_back(esterel_plan_msg);

        best_plan_ = ordered_nodes_;

        action_to_be_executed_ = best_plan_[0];

        double plan_success_probability;

        plan_success_probability = getCurrentPlanProbabilityAndFillExpectedFacts();
        exec_aternatives_msg_.plan_success_prob.push_back(plan_success_probability);

        // ROS_INFO("---------------------");
        // ROS_INFO(">>> SOLUTION FOUND <<<");
        // printNodes("Plan", ordered_nodes_);
        // ROS_INFO("Probability: %f", plan_success_probability);

        // backtrack: popf, remove last element from f, store in variable and revert that action
        backtrack("Solution found", true);

        return true;
    }
    else
        ROS_DEBUG("goals not achieved yet");

    ROS_DEBUG("finding valid nodes from open list now");
    std::vector<int> valid_nodes;
    validNodes(open_list, valid_nodes);
    if(valid_nodes.size() == 0) {
        ROS_DEBUG("valid nodes are empty");
        // ROS_INFO("$$$ No valid nodes $$$");
        // backtrack: popf, remove last element from f, store in variable and revert that action
        backtrack("Valid nodes is empty", true);
        return false;
    }
    else
        ROS_DEBUG("valid nodes search has finished: found valid nodes");


    // iterate over actions in valid nodes (V)
    for(auto a=valid_nodes.begin(); a!=valid_nodes.end(); a++) {

        // ROS_INFO("--------------------------------");
        // printNodes("Plan", ordered_nodes_);
        // // Print valid nodes
        // printNodes("Valid nodes", valid_nodes);

        std::vector<int> open_list_copy = open_list;

        // get action properties (name, params, type) from node id
        std::string action_name;
        std::vector<std::string> params;
        bool action_start;
        int action_id;
        if(!getAction(*a, action_name, params, original_plan_, action_start, action_id)) {
            ROS_ERROR("failed to get action properties (while applying action)");
            return false;
        }

        std::string full_action_name = getFullActionName(*a);
        // ROS_INFO("Action: %s", full_action_name.c_str());

        // remove a (action) and s (skipped nodes) from open list (O)
        open_list_copy.erase(std::remove(open_list_copy.begin(), open_list_copy.end(), *a), open_list_copy.end());

        ///////////////////////////////////////////////////////
        rosplan_dispatch_msgs::CalculateActionsProbabilityService srv;
        srv.request.node = full_action_name;
        double plan_success_probability;
        if(calculate_actions_prob_client_.call(srv)){
            plan_success_probability = srv.response.plan_success_probability;
            // ROS_INFO("ISR: (%s) Received response: %f", ros::this_node::getName().c_str(), plan_success_probability);
        }
        else{
            // ROS_INFO("ISR: (%s) Did NOT receive response", ros::this_node::getName().c_str());
        }

        int size = exec_aternatives_msg_.plan_success_prob.size();
        double best_prob_yet = 0;
        if(size != 0)
            best_prob_yet = exec_aternatives_msg_.plan_success_prob[size-1];

        // ROS_INFO("Probability after action: %f", plan_probability);
        // ROS_INFO("Best probability yet: %f", best_prob_yet);

        ordered_nodes_.push_back(*a);

        if(!simulateAction(action_start, action_name, params))
            return false;

        if(plan_success_probability > best_prob_yet){
            number_expanded_nodes++;

            // ROS_INFO("+++ Apply action +++");
        
            // Add action to queue
            orderNodes(open_list_copy, number_expanded_nodes);
            // printNodes("stack after adding", ordered_nodes_);
        }
        else{
            // ROS_INFO("--- Skip action ---");
            backtrack("Plan probability too low", true);
        }

        ///////////////////////////////////////////////////////

        // ROS_INFO("Calling getActionsJointProbability");
        // CPyObject pFuncCalcProb = PyObject_GetAttrString(pModule_, "getActionsJointProbability");
        // if(pFuncCalcProb && PyCallable_Check(pFuncCalcProb)){
            
        //     PyObject* pArgs = PyTuple_New(1);
        //     CPyObject py_action_string = PyUnicode_FromString(full_action_name.c_str());
        //     PyTuple_SetItem(pArgs, 0, py_action_string);

        //     CPyObject pReturnedProbability = PyObject_CallObject(pFuncCalcProb, pArgs);
        //     PyErr_Print();
        //     double plan_probability = PyFloat_AsDouble(pReturnedProbability);

        //     if(plan_probability == -1.0){
        //         ROS_ERROR("Received NULL from getActionsJointProbability");
        //         PyErr_Print();
        //     }

        //     int size = exec_aternatives_msg_.plan_success_prob.size();
        //     double best_prob_yet = 0;
        //     if(size != 0)
        //         best_prob_yet = exec_aternatives_msg_.plan_success_prob[size-1];
            
        //     // ROS_INFO("Probability after action: %f", plan_probability);
        //     // ROS_INFO("Best probability yet: %f", best_prob_yet);

        //     ordered_nodes_.push_back(*a);

        //     if(!simulateAction(action_start, action_name, params))
        //         return false;

        //     if(plan_probability > best_prob_yet){
        //         number_expanded_nodes++;

        //         // ROS_INFO("+++ Apply action +++");
            
        //         // Add action to queue
        //         orderNodes(open_list_copy, number_expanded_nodes);
        //         // printNodes("stack after adding", ordered_nodes_);
        //     }
        //     else{
        //         // ROS_INFO("--- Skip action ---");
        //         backtrack("Plan probability too low", true);
        //     }
        // }
        // else{
        //     ROS_ERROR("Could not call getActionsJointProbability");
        // }
    }

    // pop last element from stack (ordered_nodes_) revert action
    backtrack("For loop ended (valid nodes exhausted)", true);
    return true;
}

std::vector<std::string> CSPExecGenerator::getNodesWithNames(std::vector<int> &nodes)
{
    std::vector<std::string> vec;
    for(auto nit=nodes.begin(); nit!=nodes.end(); nit++) {
        std::stringstream ss;
        std::string action_name;
        std::vector<std::string> params;
        bool action_start;
        int action_id;
        getAction(*nit, action_name, params, original_plan_, action_start, action_id);
        ss << action_name;
        if(action_start)
            ss << "_start";
        else
            ss << "_end";
        int size = params.size();
        for(int i=0; i<size; i++){
            ss << "#";
            ss << params[i];
        }
        vec.push_back(ss.str());
    }

    return vec;
}

bool CSPExecGenerator::stateHasFact(std::vector<rosplan_knowledge_msgs::KnowledgeItem> state, rosplan_knowledge_msgs::KnowledgeItem fact){
    for(auto&& fact_in_state: state){
        if(factsAreEqual(fact_in_state, fact))
            return true;
    }
    return false;
}

bool CSPExecGenerator::stateContainsFacts(std::vector<rosplan_knowledge_msgs::KnowledgeItem> state,
                                                    std::vector<rosplan_knowledge_msgs::KnowledgeItem> facts){
    for(auto&& fact: facts){
        if(!stateHasFact(state, fact)){
            return false;
        }
    }
    return true;
}

bool CSPExecGenerator::actionsHaveSameNameAndParams(int action1, int action2){
    bool same_name_params;
    
    std::string name1;
    std::string name2;
    std::vector<std::string> params1;
    std::vector<std::string> params2;
    bool action_start1;
    bool action_start2;
    int action_id1;
    int action_id2;

    if(!getAction(action1, name1, params1, original_plan_, action_start1, action_id1)) {
        ROS_ERROR("failed to get action properties (while checking same name and params 1");
    }

    if(!getAction(action2, name2, params2, original_plan_, action_start2, action_id2)) {
        ROS_ERROR("failed to get action properties (while checking same name and params 1");
    }

    if( name1 == name2 && params1 == params2)
        same_name_params = true;
    else
        same_name_params = false;
    
    return same_name_params;
}

bool CSPExecGenerator::atStartAlreadyExecuted(int a){

    bool at_start_executed = false;

    if(!isStartAction(a)){
        for(auto&& action: actions_occurring_){
            if(actionsHaveSameNameAndParams(a, action)){
                if(isStartAction(action)){
                    at_start_executed = true;
                    break;
                }
            }
        }
    }

    return at_start_executed;
}

int CSPExecGenerator::getActionEndLayer(int a){
    if(isStartAction(a)){
        for(int i = 0; i < best_plan_.size(); i++){
            if(actionsHaveSameNameAndParams(a, best_plan_[i])){
                if(!isStartAction(best_plan_[i])){
                    return i;
                }
            }
        }
    }
}

int CSPExecGenerator::currentStateContainsExpectedFacts(){

    // TODO: The perturbations do not appear in this current state
    std::vector<rosplan_knowledge_msgs::KnowledgeItem> current_state = action_simulator_.getCurrentState();
    ROS_INFO(">>> Current state: %s", getStateAsString(current_state).c_str());
    printNodes(">>> Best plan", best_plan_);
    printNodes(">>> Actions occurring", actions_occurring_);

    for (int i = expected_facts_.size(); i-- > 0; ){
        std::vector<rosplan_knowledge_msgs::KnowledgeItem> expected_facts = expected_facts_[i];
        bool all_facts_are_present = true;
        for(auto&& fact: expected_facts){
            if(!stateHasFact(current_state, fact)){
                all_facts_are_present = false;
                break;
            }
        }

        // Go from last layer to first one
        // When a layer matches the state of the world, then good
        // Check if there are any previous actions which have been started but not ended
        // If there are, then dispatch the corresponding action end
        // If there aren't, then dispatch action after the layer
        if(all_facts_are_present){
            if(actions_occurring_.size() > 0){
                int action = actions_occurring_[0];
                return getActionEndLayer(action);
            }
            else{
                int action = best_plan_[i];
                if(isStartAction(action) || atStartAlreadyExecuted(action)){
                    return i;
                }
            }
        }

        // // If layer has all facts then check if action after layer is at_start
        // // or if at_end but it's at_start has already been executed
        // if(all_facts_are_present){
        //     int action = best_plan_[i];
        //     // ROS_INFO(">>> Action after layer: %s", getFullActionName(action).c_str());
        //     // If action is at_start or at_start has already been executed
        //     // ROS_INFO(isStartAction(action)? "Start action: YES":"Start action: NO");
        //     // ROS_INFO(atStartAlreadyExecuted(action)? "At start executed: YES":"At start executed: NO");            
        //     if(isStartAction(action) || atStartAlreadyExecuted(action)){
        //         return i;
        //     }
        // }
    }
    
    return -1;
}

void CSPExecGenerator::reusePreviousPlan(int index_facts){
    // Create plan with only the necessary actions
    std::vector<int> plan = best_plan_;
    plan.erase(plan.begin(), plan.begin() + index_facts);

    action_to_be_executed_ = plan.front();

    // printNodes(">>> New plan: ", plan);
    ROS_INFO("//// Total number of nodes expanded: %d ////", total_number_nodes_expanded);

    // ROS_INFO(">>> Action to be executed 1: %s", getFullActionName(action_to_be_executed_).c_str());

    // convert list of orderes nodes into esterel plan (reuses the originally received esterel plan)
    rosplan_dispatch_msgs::EsterelPlan esterel_plan_msg = convertListToEsterel(plan);

    // add new valid ordering to ordered plans (R)
    exec_aternatives_msg_.esterel_plans.push_back(esterel_plan_msg);

    // double plan_success_probability = computePlanProbability(best_plan_, action_prob_map_);
    // TODO: Calculate prob
    exec_aternatives_msg_.plan_success_prob.push_back(0.4);
}


bool CSPExecGenerator::isNodeAnActionOcurring(int node){
    for(int action: actions_occurring_){
        if(node == action){
            return true;
        }
    }
    return false;
}


bool CSPExecGenerator::generatePlans()
{
    // get current state (S) and store in memory
    action_simulator_.saveKBSnapshot();

    // init open list (O), initially contains all nodes in partial order plan
    std::vector<int> open_list;
    for(auto nit=original_plan_.nodes.begin(); nit!=original_plan_.nodes.end(); nit++) { // nit=node iterator
        // do not add plan start node
        switch(nit->node_type) {
        case rosplan_dispatch_msgs::EsterelPlanNode::ACTION_START:
        case rosplan_dispatch_msgs::EsterelPlanNode::ACTION_END:
            // remove nodes which are currently/done executing from open list, you receive
            // this information from the dispatcher inside the service call request
            if(std::find(action_executing_.begin(), action_executing_.end(), nit->node_id) != action_executing_.end()) {
                ROS_DEBUG("ignoring node (%d) because is currently being/done executed", nit->node_id);
            } else {
                open_list.push_back(nit->node_id);
            }
            break;
        }
    }


    // Check if current state contains expected facts
    // It starts from the end of expected facts because the state might
    // have unexpectedly changed and we may be able to skip actions
    ROS_INFO("Checking expected facts");
    if(!expected_facts_.empty()){
        int index_facts = currentStateContainsExpectedFacts();
        // ROS_INFO(">>> Layer selected: %d", index_facts);
        if(index_facts >= 0){
            ROS_INFO(">>> Reusing plan");
            reusePreviousPlan(index_facts);
            return true;
        }
    }

    ROS_INFO("Building new plan");

    std::vector<int> open_list_copy = open_list;
    // printNodes("Open list before", open_list);
    // printNodes("Actions ocurring", actions_occurring_);
    bool all_elements_removed = false;
    while(!all_elements_removed){
        for(int i = 0; i < open_list.size(); i++){
            if(isNodeAnActionOcurring(open_list[i])){
                open_list.erase(open_list.begin()+i);
                break;
            }
        }
        all_elements_removed = true;
    }
    // printNodes("Open list after ", open_list);

    // init set of constraints (C)
    initConstraints(set_of_constraints_);

    // init set of ordered nodes (F)
    ordered_nodes_.clear();
    // NOTE: init set of totally ordered plans (R) is stored in exec_aternatives_msg_.esterel_plans

    std::vector<std::vector<rosplan_knowledge_msgs::KnowledgeItem>> explored_states;
    int number_expanded_nodes = 0;

    ROS_INFO("Total number of actions: %d", (int)open_list.size());

    // CPyInstance hInstance;

    // ROS_INFO("ISR: (%s) Add current directory to system path", ros::this_node::getName().c_str());
    // PyObject* sysPath = PySys_GetObject((char*)"path");
    // PyList_Append(sysPath, PyUnicode_FromString("/home/tomas/ros_ws/src/ROSPlan/src/rosplan/rosplan_planning_system/src/PlanDispatch/"));

    // ROS_INFO("ISR: (%s) Setting arguments", ros::this_node::getName().c_str());
    // int argc = 1;
    // char *argv[1];
    // PySys_SetArgv(argc, argv);

    // ROS_INFO("ISR: (%s) Importing rospy", ros::this_node::getName().c_str());
    // PyObject *rospy = PyImport_ImportModule("rospy");
    // if (!rospy){
    //     ROS_ERROR("ERROR: (%s) Failed to import rospy", ros::this_node::getName().c_str());
    //     PyErr_Print();
    // }

    // ROS_INFO("ISR: (%s) Getting rospy init_node", ros::this_node::getName().c_str());
    // PyObject *init_node = PyObject_GetAttrString(rospy, "init_node");
    // if (!init_node){
    //     ROS_ERROR("ERROR: (%s) Failed to initialise rospy node", ros::this_node::getName().c_str());
    //     PyErr_Print();
    // }

    // ROS_INFO("ISR: (%s) Initiliasing rospy node", ros::this_node::getName().c_str());
    // PyObject *init_node_args = PyTuple_New(1);
    // PyObject *node_name = PyString_FromString("bayes_net_calc");	
    // PyTuple_SetItem(init_node_args, 0, node_name);
    // PyObject_CallObject(init_node, init_node_args);

    // ROS_INFO("ISR: (%s) Importing calculate_plan_prob", ros::this_node::getName().c_str());
    // CPyObject pName = PyUnicode_FromString("calculate_plan_prob");
    // pModule_ = PyImport_Import(pName);

    // if(pModule_){

    //     ROS_INFO("Calling setup method");
    //     CPyObject pFuncSetup = PyObject_GetAttrString(pModule_, "setup");
    //     if(pFuncSetup && PyCallable_Check(pFuncSetup)){
    //         PyObject_CallObject(pFuncSetup, NULL);

    //         // find plan
    //         // if true, it means at least one valid execution alternative was found
    //         ROS_INFO("Calling orderNodes");
    //         orderNodes(open_list, number_expanded_nodes);

    //     }
    //     else{
    //         PyErr_Print();
    //         ROS_ERROR("ERROR: Failed to call setup");
    //     }

    // }
    // else{
    //     PyErr_Print();
    //     ROS_ERROR("ERROR: Failed to call calculate_plan_prob");
    // }

    rosplan_dispatch_msgs::SetupService srv;
    if(setup_client_.call(srv)){
        if(srv.response.success){
            // ROS_INFO("ISR: (%s) Successfully called setup method", ros::this_node::getName().c_str());
        }
        else{
            // ROS_INFO("ISR: (%s) Something went wrong in setup method", ros::this_node::getName().c_str());
        }
    }
    else{
        // ROS_INFO("ISR: (%s) Couldn't call setup method", ros::this_node::getName().c_str());
    }

    orderNodes(open_list, number_expanded_nodes);

    // ROS_INFO("#### Number of nodes expanded: %d ####", number_expanded_nodes);
    total_number_nodes_expanded += number_expanded_nodes;
    // ROS_INFO("//// Total number of nodes expanded: %d ////", total_number_nodes_expanded);
    // ROS_INFO("|||| Average service call time: %f", average_service_time);
    return (exec_aternatives_msg_.esterel_plans.size()>0);
}

bool CSPExecGenerator::getEdgeFromEdgeID(int edge_id, rosplan_dispatch_msgs::EsterelPlan &esterel_plan,
        rosplan_dispatch_msgs::EsterelPlanEdge &edge)
{
    // iterate over the edges of the plan
    for(auto eit=esterel_plan.edges.begin(); eit!=esterel_plan.edges.end(); eit++) {
        // compare edge id to the received input id
        if(eit->edge_id == edge_id) {
            // return edge by reference
            edge = *eit;
            return true;
        }
    }

    ROS_ERROR("could not found matching edge from given edge id : (%d)", edge_id);
    return false;
}

rosplan_dispatch_msgs::EsterelPlan CSPExecGenerator::removeConditionalEdges(
        rosplan_dispatch_msgs::EsterelPlan &esterel_plan, std::vector<int> &ordered_nodes)
{
    // remove all conditional edges in a esterel plan, plan is received and modified by reference

    rosplan_dispatch_msgs::EsterelPlan output_plan;

    std::vector<int> skipped_nodes; // keep track of skipped nodes
    // iterate over originally received esterel_plan and copy all nodes
    for(auto nit=esterel_plan.nodes.begin(); nit!=esterel_plan.nodes.end(); nit++) {
        // check if node id belongs to ordered_nodes
        if(std::find(ordered_nodes.begin(), ordered_nodes.end(), nit->node_id) != ordered_nodes.end()) {
            // found node id in ordered_nodes, add to plan
            // output_plan.nodes.push_back(*nit);

            rosplan_dispatch_msgs::EsterelPlanNode node_msg;
            node_msg.node_type = nit->node_type;
            node_msg.node_id = nit->node_id;
            node_msg.name = nit->name;
            node_msg.action = nit->action;

            // the following 2 for loops will basically do:
            // node_msg.edges_out = nit->edges_out;
            // node_msg.edges_in = nit->edges_in;
            // but without conditional edges

            /*/ iterate over nit->edges_out and ensure they are not conditional edges
            for(auto reit=nit->edges_out.begin(); reit!=nit->edges_out.end(); reit++) {
                // get edge from edge id
                rosplan_dispatch_msgs::EsterelPlanEdge is_conditional_edge_question;
                if(getEdgeFromEdgeID(*reit, esterel_plan, is_conditional_edge_question)) {
                    if(is_conditional_edge_question.edge_type != rosplan_dispatch_msgs::EsterelPlanEdge::CONDITION_EDGE)
                        node_msg.edges_out.push_back(*reit);
                }
            }*/

            /*/ iterate over nit->edges_in and ensure they are not conditional edges
            for(auto reit=nit->edges_in.begin(); reit!=nit->edges_in.end(); reit++) {
                // get edge from edge id
                rosplan_dispatch_msgs::EsterelPlanEdge is_conditional_edge_question;
                if(getEdgeFromEdgeID(*reit, esterel_plan, is_conditional_edge_question)) {
                    if(is_conditional_edge_question.edge_type != rosplan_dispatch_msgs::EsterelPlanEdge::CONDITION_EDGE)
                        node_msg.edges_in.push_back(*reit);
                }
            }*/

            output_plan.nodes.push_back(node_msg);
        }
        else {
            // skipped node, keep a list of them
            skipped_nodes.push_back(nit->node_id);
        }
    }

    /*/ iterate over the edges
    for(auto eit=esterel_plan.edges.begin(); eit!=esterel_plan.edges.end(); eit++) {
        // skip conditional edges
        if(eit->edge_type != rosplan_dispatch_msgs::EsterelPlanEdge::CONDITION_EDGE) {
            // copy edge properties from original plan
            rosplan_dispatch_msgs::EsterelPlanEdge edge;
            edge.edge_type = eit->edge_type;
            edge.edge_id = eit->edge_id;
            edge.edge_name = eit->edge_name;
            edge.signal_type = eit->signal_type;
            edge.duration_lower_bound = eit->duration_lower_bound;
            edge.duration_upper_bound = eit->duration_upper_bound;

            */ /*/ remove skipped nodes
            for(auto sourceit=eit->source_ids.begin(); sourceit!=eit->source_ids.begin(); sourceit++) {
                // check if node id belongs to ordered_nodes
                if(!(std::find(skipped_nodes.begin(), skipped_nodes.end(), *sourceit) != skipped_nodes.end())) {
                    // did not found node id in skipped_nodes
                    edge.source_ids.push_back(*sourceit);
                }
            }*/

            /*/ remove skipped nodes
            for(auto sinkit=eit->sink_ids.begin(); sinkit!=eit->sink_ids.begin(); sinkit++) {
                // check if node id belongs to ordered_nodes
                if(!(std::find(skipped_nodes.begin(), skipped_nodes.end(), *sinkit) != skipped_nodes.end())) {
                    // did not found node id in skipped_nodes
                    edge.sink_ids.push_back(*sinkit);
                }
            }

            output_plan.edges.push_back(edge);
        }
    }*/

    return output_plan;
}

rosplan_dispatch_msgs::EsterelPlan CSPExecGenerator::convertListToEsterel(std::vector<int> &ordered_nodes)
{
    // add constrains to the partially ordered plan (esterel plan without conditional edges)
    // remove nodes which are not in ordered_nodes (skipped nodes) from plan

    // remove conditional edges from plan and skipped nodes
    rosplan_dispatch_msgs::EsterelPlan esterel_plan = removeConditionalEdges(original_plan_, ordered_nodes);

    // NOTE: it is assumed that input plan already does not has conditional edges and is therefore a partial order plan
    // add edges to esterel_plan, they are added as conditional (hack), they are not conditional edges

    // keep memory of the last edge for naming future edges
    rosplan_dispatch_msgs::EsterelPlanEdge last_edge = original_plan_.edges.back();
    int edge_id_count = last_edge.edge_id;

    // iterate over the ordered nodes
    for(auto nit=ordered_nodes.begin(); nit<(ordered_nodes.end() - 1); nit++) {
        // assume each pair of nodes is a constraint

        // e.g. [1,3,2,4,5,6] -> edge(1,3), edge(3,2), edge(2,4), edge(4,5), edge(5,6)
        // where edge(1,3) source id = 1, sink id = 3, node 1 -> edges_out = edge id, node 3 -> edges in = edge id

        // create empty edge msg
        rosplan_dispatch_msgs::EsterelPlanEdge edge_msg;

        // fill edge msg
        edge_msg.edge_type = rosplan_dispatch_msgs::EsterelPlanEdge::CONDITION_EDGE;
        edge_msg.edge_id = ++edge_id_count;
        std::string new_edge_name = "edge_" + std::to_string(edge_id_count);
        edge_msg.edge_name = new_edge_name;
        edge_msg.signal_type = 0;
        edge_msg.source_ids.push_back(*nit);
        edge_msg.sink_ids.push_back(*(nit + 1));
        edge_msg.duration_lower_bound = 0.0;
        edge_msg.duration_upper_bound = 0.0;

        // iterate over the nodes in the plan, to add edges (needed by the dispatcher)
        for(auto pnit=esterel_plan.nodes.begin(); pnit!=esterel_plan.nodes.end(); pnit++) {
            if(pnit->node_id == *nit) pnit->edges_out.push_back(edge_msg.edge_id);
            if(pnit->node_id == *(nit+1)) pnit->edges_in.push_back(edge_msg.edge_id);
        }

        // add edge (as conditional edge -> workaround)
        esterel_plan.edges.push_back(edge_msg);
    }

    return esterel_plan;
}

bool CSPExecGenerator::isStartAction(int a){
    std::string action_name;
    std::vector<std::string> params;
    bool action_start;
    int action_id;
    if(!getAction(a, action_name, params, original_plan_, action_start, action_id)) {
        ROS_ERROR("failed to get action properties (while getting name");
    }
    return action_start;

}

void CSPExecGenerator::removeStartActionFromOccurringActions(int action1){
    if(!isStartAction(action1)){
        int i;
        for(i = 0; i<actions_occurring_.size(); i++){
            if(actionsHaveSameNameAndParams(action1, actions_occurring_[i])){
                if(actions_occurring_[i]){
                    break;
                }
            }
        }
        actions_occurring_.erase(actions_occurring_.begin()+i);
    }
}

bool CSPExecGenerator::srvCB(rosplan_dispatch_msgs::ExecAlternatives::Request& req, rosplan_dispatch_msgs::ExecAlternatives::Response& res)
{
    ROS_INFO("\n");
    ROS_INFO("generating execution alternatives service is computing now");

    if(!is_esterel_plan_received_) {
        // esterel plan not received yet!
        ROS_ERROR("Generation of plan alternatives requires an esterel plan as input but it has not being received yet");

        // replanning is needed, to enforce reveiving the esterel plan
        res.replan_needed = true;

        // indicate that no valid execution was found
        res.exec_alternatives_generated = false;

        // service call was succesful (regardless if at least one plan was found or not)
        return true;
    }

    // lower flag to force the node to receive a new plan if a new request comes
    // is_esterel_plan_received_ = false;

    // save nodes which are being/done executing in member variable to be removed from open list (skipped)
    action_executing_ = req.actions_executing;

    // delete old data if any
    exec_aternatives_msg_.esterel_plans.clear();
    exec_aternatives_msg_.plan_success_prob.clear();

    if(generatePlans()) // compute exec alternatives
    {
        // indicates that at least one valid execution was found
        res.replan_needed = false;
        res.exec_alternatives_generated = true;
        ROS_INFO("Found %ld valid execution(s)", exec_aternatives_msg_.esterel_plans.size());

        // plans could be printed here for debugging purposes

        // publish esterel array msg
        pub_valid_plans_.publish(exec_aternatives_msg_);

        ROS_INFO(">>> Action to be executed: %s", getFullActionName(action_to_be_executed_).c_str());

        if(isStartAction(action_to_be_executed_)){
            actions_occurring_.push_back(action_to_be_executed_);
        }
        else if(atStartAlreadyExecuted(action_to_be_executed_)){
            removeStartActionFromOccurringActions(action_to_be_executed_);
        }
    }
    else
    {
        // indicates that no valid execution was found, means replanning is needed
        res.replan_needed = true;
        res.exec_alternatives_generated = false;
        ROS_INFO("No valid execution was found, replanning is needed");
    }

    ROS_INFO("Generating execution alternatives service has finished");

    return true;
}

void CSPExecGenerator::update()
{
    // check if callbacks have requests
    ros::spinOnce();
}

int main(int argc, char **argv)
{
    // init node
    ros::init(argc, argv, "csp_exec_generator_node");
    ROS_DEBUG("node is going to initialize...");

    // create object of the node class (CSPExecGenerator)
    CSPExecGenerator csp_exec_generator_node;

    // setup node frequency
    double node_frequency = 0.0;
    ros::NodeHandle nh("~");
    nh.param("node_frequency", node_frequency, 10.0);
    ROS_DEBUG("Node will run at : %lf [hz]", node_frequency);
    ros::Rate loop_rate(node_frequency);

    ROS_DEBUG("Node initialized.");

    // useful for debugging without having to call service, uncomment if needed
    // csp_exec_generator_node.testFunctions();

    while (ros::ok())
    {
        // main loop function
        csp_exec_generator_node.update();

        // sleep to control the node frequency
        loop_rate.sleep();
    }

    return 0;
}
