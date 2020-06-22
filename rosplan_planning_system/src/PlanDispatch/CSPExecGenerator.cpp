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
#include <chrono>
#include <algorithm>

int total_number_nodes_expanded = 1;
bool branch_and_bound;
int number_service_calls = 0;
// double service_time_sum = 0;

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
    calculate_prob_client_ = nh.serviceClient<rosplan_dispatch_msgs::CalculateProbability>("calculate_plan_probability");
    
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
        std::stringstream full_name = getFullActionName(action_name, params, action_start);
        ss << full_name.str();
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

void CSPExecGenerator::backtrack(std::string reason_for_backtrack)
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
        ss << " || ";
    }
    return ss.str();
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
            if(action_simulator_.convertPredToString(fact1) == action_simulator_.convertPredToString(fact2)){
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

std::stringstream CSPExecGenerator::getFullActionName(std::string action_name, std::vector<std::string> params, bool action_start){
    std::stringstream ss;
    ss << action_name;
    if(action_start){
        ss << "_start";
    }
    else{
        ss << "_end";
    }
    for(auto&& parameter: params) {
        ss << "%";
        ss << parameter;
    }

    return ss;
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

bool CSPExecGenerator::stateIsRepeated(std::vector<rosplan_knowledge_msgs::KnowledgeItem> state,
                                        std::vector<std::vector<rosplan_knowledge_msgs::KnowledgeItem>> explored_states){
    bool repeated_state = false;
    for(auto&& state_explored: explored_states){
        if(statesAreEqual(state, state_explored)){
            return true;
        }
    }
    return false;
}

std::vector<rosplan_knowledge_msgs::KnowledgeItem> CSPExecGenerator::getStateAfterAction(std::string action_name,
                                                                                        std::vector<std::string> params,
                                                                                        std::vector<int> open_list){

    std::string action1 = getFullActionName(action_name, params, true).str();

    for(auto a=open_list.begin(); a!=open_list.end(); a++){
        // get action properties (name, params, type) from node id
        std::string name;
        std::vector<std::string> parameters;
        bool action_start;
        int action_id;
        if(!getAction(*a, name, parameters, original_plan_, action_start, action_id)) {
            ROS_ERROR("failed to get action properties (while applying action)");
        }
        
        std::string action2 = getFullActionName(name, parameters, action_start).str();

        if(action1 == action2){
            // ROS_INFO("Found action");
            action1 = getFullActionName(action_name, params, false).str();
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

    backtrack("Got state after action 1");
    backtrack("Got state after action 2");

    return state_after_action;
}


bool CSPExecGenerator::orderNodes(std::vector<int> open_list, int &number_expanded_nodes, double plan_prob,
                                    std::vector<std::vector<rosplan_knowledge_msgs::KnowledgeItem>> explored_states)
{
    // shift nodes from open list (O) to ordered plans (R)
    // offering all possible different execution alternatives via DFS (Depth first search)

    ROS_DEBUG("order nodes (recurse)");

    // Add current state to explored_states
    std::vector<rosplan_knowledge_msgs::KnowledgeItem> current_state = action_simulator_.getCurrentState();
    explored_states.push_back(current_state);

    if(!checkTemporalConstraints(ordered_nodes_, set_of_constraints_)) {
        // ROS_INFO("$$$ Temporal constraints not satisfied $$$$");
        backtrack("temporal constraints not satisfied");
        return false;
    }

    // check if goals are achieved
    ROS_DEBUG("checking if goals are achieved...");
    if(action_simulator_.areGoalsAchieved()) {
        // we print all plans at the end, so only we print here in debug mode
        //ROS_INFO("found valid ordering:");
        // printNodes("plan", ordered_nodes_);

        // convert list of orderes nodes into esterel plan (reuses the originally received esterel plan)
        rosplan_dispatch_msgs::EsterelPlan esterel_plan_msg = convertListToEsterel(ordered_nodes_);

        // add new valid ordering to ordered plans (R)
        exec_aternatives_msg_.esterel_plans.push_back(esterel_plan_msg);

        best_plan_ = ordered_nodes_;

        // compute plan probability
        rosplan_dispatch_msgs::CalculateProbability srv;
        double plan_success_probability;
        srv.request.nodes = ordered_nodes_;
        if(calculate_prob_client_.call(srv)){
            plan_success_probability = srv.response.plan_success_probability;
        }
        else{
            plan_success_probability = computePlanProbability(ordered_nodes_, action_prob_map_);
        }
        exec_aternatives_msg_.plan_success_prob.push_back(plan_success_probability);

        // ROS_INFO(">>> Goal achieved with probability %f <<<\n", plan_success_probability);

        expected_state_ = state_after_first_action_;

        // backtrack: popf, remove last element from f, store in variable and revert that action
        backtrack("goal was achieved");

        return true;
    }
    else
        ROS_DEBUG("goals not achieved yet");

    // cap the maximum amount of plans to generate
    if(exec_aternatives_msg_.esterel_plans.size() > max_search_depth_) {
        ROS_DEBUG("returning early : max amount of plans reached (%ld)", exec_aternatives_msg_.esterel_plans.size());
        // ROS_INFO("$$$ Maximum plan size reached $$$");
        backtrack("We do not want to search deeper");
        return true;
    }

    ROS_DEBUG("finding valid nodes from open list now");
    std::vector<int> valid_nodes;
    validNodes(open_list, valid_nodes);
    if(valid_nodes.size() == 0) {
        ROS_DEBUG("valid nodes are empty");
        // ROS_INFO("$$$ No valid nodes $$$");
        // backtrack: popf, remove last element from f, store in variable and revert that action
        backtrack("nodes are empty");
        return false;
    }
    else
        ROS_DEBUG("valid nodes search has finished: found valid nodes");

    // iterate over actions in valid nodes (V)
    for(auto a=valid_nodes.begin(); a!=valid_nodes.end(); a++) {

        // Print valid nodes
        // std::stringstream ss = getNodesWithNames(valid_nodes);
        // ROS_INFO("[ Valid Nodes: %s ]", ss.str().c_str());

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

        branch_and_bound = true;
        bool repeated_state = false;

        if(action_start){
            std::vector<rosplan_knowledge_msgs::KnowledgeItem> state_after_action = getStateAfterAction(action_name, params, open_list_copy);
            repeated_state = stateIsRepeated(state_after_action, explored_states);
            if(ordered_nodes_.size() == 0)
                state_after_first_action_ = state_after_action;
            // ROS_INFO(">>> State after action: %s", getStateAsString(state_after_first_action).c_str());
            // ROS_INFO(">>> Current state: %s", getStateAsString(current_state).c_str());
            // ROS_INFO(repeated_state ? "+++ State is repeated +++" : "--- State is NOT repeated");
        }

        // remove a (action) and s (skipped nodes) from open list (O)
        open_list_copy.erase(std::remove(open_list_copy.begin(), open_list_copy.end(), *a), open_list_copy.end());

        // repeated_state = false;
        if(branch_and_bound){
            if(!repeated_state){
            
            double plan_success_probability;
            std::chrono::steady_clock::time_point call_time = std::chrono::steady_clock::now();

            // if(calculate_prob_client_.call(srv)){
            //     std::chrono::steady_clock::time_point response_time = std::chrono::steady_clock::now();
            //     double full_service_time = std::chrono::duration_cast<std::chrono::nanoseconds> (response_time - call_time).count() * (double)pow(10,-9);
            //     // double computing_service_time = srv.response.computing_time;
            //     // service_time_sum += full_service_time - computing_service_time;
            //     number_service_calls++;
            //     // ROS_INFO("|||| Service time: %f s|||", service_time_s);
            //     plan_success_probability = srv.response.plan_success_probability;
            //     // ROS_INFO("||| Received response: %f |||", plan_success_probability);
            // }
            // else{
                // ROS_INFO("||| DID NOT RECEIVE RESPONSE |||");
                std::map<int, double>::const_iterator prob_it = action_prob_map_.find(*a);
                double action_prob = prob_it->second;
                plan_success_probability = plan_prob*action_prob;
                // plan_success_probability = computePlanProbability(ordered_nodes_, action_prob_map_);
            // }

            // TODO: Save length of plan and save the shortest one with the highest success probability
            //// If success probability is the same, save it if the number of actions is lower
            int size = exec_aternatives_msg_.plan_success_prob.size();
            double best_prob_yet = 0;
            if(size != 0)
                best_prob_yet = exec_aternatives_msg_.plan_success_prob[size-1];
            
            // ROS_INFO(">>> Current probability: %f", plan_success_probability);
            // ROS_INFO(">>> Best probability yet: %f", best_prob_yet);

            if(plan_success_probability > best_prob_yet){
                number_expanded_nodes++;

                // std::stringstream full_action_name = getFullActionName(action_name, params, action_start);
                // ROS_INFO(">>>>> Apply action : %s <<<<<", full_action_name.str().c_str());
                // Add action to queue
                ordered_nodes_.push_back(*a);

                if(!simulateAction(action_start, action_name, params))
                    return false;
               
                orderNodes(open_list_copy, number_expanded_nodes, plan_success_probability, explored_states);

                // printNodes("stack after adding", ordered_nodes_);

                // ROS_INFO("++++ Performed action");
                // printNodes("stack after adding", ordered_nodes_);
            }
            else{
                // ROS_INFO("---- Skipped action");
                // printNodes("stack after adding", ordered_nodes_);
                // backtrack("success probability lower than best one so far");
            }
            
        }

        }
        else{
            //// Original version on code ////
            number_expanded_nodes++;
            // ROS_INFO(">>>>> Apply action : (%d)", *a);
            ordered_nodes_.push_back(*a);

            // ROS_INFO("++++ Performed action");

            // Simulate action
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

            // printNodes("stack after adding", ordered_nodes_);

            // recurse
            orderNodes(open_list_copy, number_expanded_nodes, 1, explored_states);
        }
    }

    // pop last element from stack (ordered_nodes_) revert action
    backtrack("for loop ended (valid nodes exhausted)");
    return true;
}

void CSPExecGenerator::reverseLastAction(std::string reason_for_reverse)
{
    // backtrack: popf, remove last element from f, store in variable and revert that action
    ROS_DEBUG("backtrack because %s", reason_for_reverse.c_str());
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
                action_simulator_.simulateActionStart(action_name, params);
            else
                action_simulator_.simulateActionEnd(action_name, params);

            // action_simulator_.printInternalKBFacts();
        }
        else
            ROS_ERROR("failed to get action properties (while reversing action)");
    }
}

std::stringstream CSPExecGenerator::getNodesWithNames(std::vector<int> &nodes)
{
    std::stringstream ss;
    for(auto nit=nodes.begin(); nit!=nodes.end(); nit++) {
        if(nit != nodes.begin()){
            ss << " | ";
        }
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
            ss << "%";
            ss << params[i];
        }
    }

    return ss;
}

bool CSPExecGenerator::generatePlans()
{
    ROS_INFO("\n");
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

    printNodes(">>> Open list", open_list);

    // Check if current state is the same as expected state
    if(!expected_state_.empty()){
        ROS_INFO(">>> Expected state: %s", getStateAsString(expected_state_).c_str());
        ROS_INFO(">>> Current state:  %s", getStateAsString(action_simulator_.getCurrentState()).c_str());
        ROS_INFO("States are equal: %s", statesAreEqual(expected_state_, action_simulator_.getCurrentState())? "YES" : "NO");
        if(statesAreEqual(expected_state_, action_simulator_.getCurrentState())){
            // discard first action (already executed)
            best_plan_.erase(best_plan_.begin());

            // convert list of orderes nodes into esterel plan (reuses the originally received esterel plan)
            rosplan_dispatch_msgs::EsterelPlan esterel_plan_msg = convertListToEsterel(best_plan_);

            // add new valid ordering to ordered plans (R)
            exec_aternatives_msg_.esterel_plans.push_back(esterel_plan_msg);

            double plan_success_probability = computePlanProbability(best_plan_, action_prob_map_);
            exec_aternatives_msg_.plan_success_prob.push_back(plan_success_probability);

            return true;
        }
    }

    // printNodes("open list", open_list);
    // printNodesWithNames(open_list);

    // init set of constraints (C)
    initConstraints(set_of_constraints_);

    // init set of ordered nodes (F)
    ordered_nodes_.clear();

    // NOTE: init set of totally ordered plans (R) is stored in exec_aternatives_msg_.esterel_plans

    std::vector<std::vector<rosplan_knowledge_msgs::KnowledgeItem>> explored_states;
    int number_expanded_nodes = 0;

    ROS_INFO("Total number of actions: %d", (int)open_list.size());

    // find plan
    // if true, it means at least one valid execution alternative was found
    orderNodes(open_list, number_expanded_nodes, 1.0, explored_states);
    // ROS_INFO("#### Number of nodes expanded: %d ####", number_expanded_nodes);
    total_number_nodes_expanded += number_expanded_nodes;
    ROS_INFO("//// Total number of nodes expanded: %d ////", total_number_nodes_expanded);
    // double average_service_time = service_time_sum/(double)number_service_calls;
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

bool CSPExecGenerator::srvCB(rosplan_dispatch_msgs::ExecAlternatives::Request& req, rosplan_dispatch_msgs::ExecAlternatives::Response& res)
{
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

        int a = best_plan_[0];
        std::string action_name;
        std::vector<std::string> params;
        bool action_start;
        int action_id;
        if(!getAction(a, action_name, params, original_plan_, action_start, action_id)) {
            ROS_ERROR("failed to get action properties (while applying action)");
            return false;
        }
        ROS_INFO(">>> Action to be executed: %s", getFullActionName(action_name, params, action_start).str().c_str());
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
