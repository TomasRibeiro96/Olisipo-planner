#include "rosplan_planning_system/PlanDispatch/AdaptablePlanDispatcher.h"
#include <unistd.h>
#include <fstream>


namespace KCL_rosplan {

	/*-------------*/
	/* constructor */
	/*-------------*/

	AdaptablePlanDispatcher::AdaptablePlanDispatcher(ros::NodeHandle& nh): display_edge_type_(false),
        PlanDispatcher(nh)  {

		node_handle = &nh;

		ros::NodeHandle nh_;
		perturb_client_ = nh_.serviceClient<rosplan_knowledge_msgs::PerturbStateService>("perturb_state");

		std::string plan_graph_topic = "plan_graph";
		nh.getParam("plan_graph_topic", plan_graph_topic);
		plan_graph_publisher = node_handle->advertise<std_msgs::String>(plan_graph_topic, 1000, true);

        gen_alternatives_client = node_handle->serviceClient<rosplan_dispatch_msgs::ExecAlternatives>("/csp_exec_generator/gen_exec_alternatives");

		// display edge type with colors (conditional edge, interference edge, etc)
		nh.param("display_edge_type", display_edge_type_, false);

		need_to_replan = false;

		std::string probabilities_file;
		nh.getParam("probabilities_file", probabilities_file);
		fillProbabilitiesMap(probabilities_file);

		action_counter_client_ = nh_.serviceClient<std_srvs::Trigger>("increment_action_count");

		number_actions_dispatched = 0;

		reset();
	}

	AdaptablePlanDispatcher::~AdaptablePlanDispatcher()
	{

	}

	void AdaptablePlanDispatcher::reset() {
		PlanDispatcher::reset();
        actions_executing.clear();
		finished_execution = true;
	}

	/*-------------------*/
	/* Plan subscription */
	/*-------------------*/

	void AdaptablePlanDispatcher::planCallback(const rosplan_dispatch_msgs::EsterelPlanArray plan) {
		// ROS_INFO("ISR: (%s) Inside planCallback", ros::this_node::getName().c_str());
		if(plan.plan_success_prob.size() != plan.esterel_plans.size()) {
			ROS_WARN("KCL: (%s) Plans received, but probabilities array of different size. Ignoring.", ros::this_node::getName().c_str());
			replan_requested = true;
			return;
		}
		if(plan.esterel_plans.size() == 0) {
			ROS_WARN("KCL: (%s) Zero plans received in message.", ros::this_node::getName().c_str());
			replan_requested = true;
			return;
		}
		float bestProb = plan.plan_success_prob[0];
		current_plan = plan.esterel_plans[0];

		current_plan = plan.esterel_plans.back();
		bestProb = plan.plan_success_prob.back();
		// for(int i=1; i<plan.plan_success_prob.size(); i++) {
		// 	if(plan.plan_success_prob[i] > bestProb) {
		// 		// highest probability
		// 		bestProb = plan.plan_success_prob[i];
		// 		current_plan = plan.esterel_plans[i];
		// 	} else if(plan.plan_success_prob[i] == bestProb && current_plan.nodes.size() < plan.esterel_plans[i].nodes.size()) { 
		// 		// break ties on number of actions
		// 		current_plan = plan.esterel_plans[i];
		// 	}
		// }

/*            std::vector<int> ac;
            for(int i=0; i<actions_executing.size(); i++) {
                if(action_completed[actions_executing[i]]) ac.push_back(actions_executing[i]);
            }
            initialise();
            for(int i=0; i<actions_executing.size(); i++) {
                
                action_dispatched[actions_executing[i]] = true;
                action_received[actions_executing[i]] = true;
                action_completed[actions_executing[i]] = (std::find(ac.begin(), ac.end(), actions_executing[i]) != ac.end());
            }
*/


		    for(std::vector<rosplan_dispatch_msgs::EsterelPlanEdge>::const_iterator ci = current_plan.edges.begin(); ci != current_plan.edges.end(); ci++) {
			    edge_active[ci->edge_id] = false;
		    }

			// ROS_INFO("KCL: (%s) Plan selected with probability %f.", ros::this_node::getName().c_str(), bestProb);
			plan_received = true;
			// printPlan();
//		} else {
//			ROS_INFO("KCL: (%s) Plan received, but current execution not yet finished.", ros::this_node::getName().c_str());
//		}
	}

	void AdaptablePlanDispatcher::perturbWorldState(){
		//// Perturb state
		rosplan_knowledge_msgs::PerturbStateService srv;
		ROS_INFO("ISR: (%s) Perturbing world state", ros::this_node::getName().c_str());
		if(perturb_client_.call(srv)){
			if(srv.response.success){
				// ROS_INFO("ISR: (%s) Successfully perturbed state", ros::this_node::getName().c_str());
			}
			else{
				// ROS_INFO("ISR: (%s) Something went wrong while perturbing", ros::this_node::getName().c_str());
			}
		}
		else{
			ROS_INFO("ISR: (%s) Did NOT perturb state", ros::this_node::getName().c_str());
		}

	}

	std::string AdaptablePlanDispatcher::getFullActionName(rosplan_dispatch_msgs::EsterelPlanNode node){
		std::stringstream ss;

		ss << node.action.name;

		if(node.node_type == node.ACTION_START){
			ss << "_start";
		}
		else{
			ss << "_end";
		}

		std::vector<diagnostic_msgs::KeyValue, std::allocator<diagnostic_msgs::KeyValue>> parameters = node.action.parameters;
		for(diagnostic_msgs::KeyValue param: parameters){
			ss << "#";
			ss << param.value;
		}

		return ss.str();
	}

	std::string AdaptablePlanDispatcher::getActionNameWithoutTime(rosplan_dispatch_msgs::EsterelPlanNode node){
		std::stringstream ss;

		ss << node.action.name;

		std::vector<diagnostic_msgs::KeyValue, std::allocator<diagnostic_msgs::KeyValue>> parameters = node.action.parameters;
		for(diagnostic_msgs::KeyValue param: parameters){
			ss << "#";
			ss << param.value;
		}

		return ss.str();
	}

	void AdaptablePlanDispatcher::printEsterelPlan(){
		std::stringstream ss;

		for(std::vector<rosplan_dispatch_msgs::EsterelPlanNode>::const_iterator ci = current_plan.nodes.begin(); ci != current_plan.nodes.end(); ci++) {

			rosplan_dispatch_msgs::EsterelPlanNode node = *ci;

			if(ci != current_plan.nodes.begin()){
				ss << ", ";
			}
			ss << getFullActionName(node);
		}

		ROS_INFO("ISR: (%s) Esterel plan: %s", ros::this_node::getName().c_str(), ss.str().c_str());

	}

	void AdaptablePlanDispatcher::makeOnlyNextActionApplicable(std::string next_action_name){

		int next_action_id = map_node_id[next_action_name];
		// ROS_INFO("ISR: (%s) Next action: %s %d", ros::this_node::getName().c_str(), next_action_name.c_str(), next_action_id);

		// std::string name_no_params = next_action_name.substr(0, next_action_name.find("#"));
		// std::size_t pos = 0;
		// while((pos = name_no_params.find("_")) != std::string::npos){
		// 	// 1 is because the length of "_" is 1
		// 	name_no_params.erase(0, pos + 1);
		// }

		for (std::pair<std::string, int> pair: map_node_id) {
			if(pair.first == next_action_name){
				// ROS_INFO("ISR: (%s) Setting node to false true: %s %d", ros::this_node::getName().c_str(), pair.first.c_str(), pair.second);
				action_dispatched[pair.second] = false;
				action_completed[pair.second] = true;
			}
			else{
				// ROS_INFO("ISR: (%s) Setting node to true false: %s %d", ros::this_node::getName().c_str(), pair.first.c_str(), pair.second);
				action_dispatched[pair.second] = true;
				action_completed[pair.second] = false;
			}
		}

		// int next_action_id;
		// std::vector<rosplan_dispatch_msgs::EsterelPlanNode>::const_iterator ci;
		// for(ci = current_plan.nodes.begin(); ci != current_plan.nodes.end(); ci++) {
		// 	rosplan_dispatch_msgs::EsterelPlanNode node = *ci;
		// 	if(node.action.action_id == next_action_id){
		// 		action_dispatched[node.action.action_id] = true;
		// 		action_completed[node.action.action_id] = false;
		// 	}
		// 	else{
		// 		action_dispatched[node.action.action_id] = false;
		// 		action_completed[node.action.action_id] = true;
		// 	}
		// }

		// std::vector<rosplan_dispatch_msgs::EsterelPlanNode>::const_iterator ci;
		// for(ci = current_plan.nodes.begin(); ci != current_plan.nodes.end(); ci++) {
		// 	rosplan_dispatch_msgs::EsterelPlanNode node = *ci;
		// 	if(node.node_id != next_action_id){
		// 		action_dispatched[node.node_id] = true;
		// 		action_completed[node.node_id] = false;
		// 	}
		// 	else{
		// 		action_dispatched[node.node_id] = false;
		// 		action_completed[node.node_id] = true;
		// 		}
		// 	}
		// }
		
	}

	void AdaptablePlanDispatcher::printIntBoolMap(std::map<int,bool> m, std::string msg){
		std::stringstream ss;
		for (auto const& pair: m) {
			std::string second = pair.second ? "True" : "False";
			ss << "\n{" << pair.first << ": " << second << "}";
		}
		ROS_INFO("ISR: (%s)\nMap of %s: %s", ros::this_node::getName().c_str(), msg.c_str(), ss.str().c_str());
	}

	void AdaptablePlanDispatcher::printVectorInts(std::vector<int> vec, std::string str){
		std::stringstream ss;
		ss << "[";
		for(int item: vec){
			if(item == *vec.begin())
				ss << "'" << item << "'";
			else{
				ss << ", '" << item << "'";
			}
		}
		ss << "]";
		ROS_INFO("ISR: (%s) %s:%s", ros::this_node::getName().c_str(), str.c_str(), ss.str().c_str());
	}

	void AdaptablePlanDispatcher::fillProbabilitiesMap(std::string probabilities_file){
		std::ifstream file;

		// ROS_INFO("ISR: (%s) Opening file: %s", ros::this_node::getName().c_str(), probabilities_file.c_str());
		file.open(probabilities_file);

		bool entering_actions_part = false;
		std::string line;

		while(getline(file, line)){
			// ROS_INFO("ISR: (%s) Line: %s", ros::this_node::getName().c_str(), line.c_str());
			if(entering_actions_part){
				int first_delimeter = line.find(" ");
				std::string action_name = line.substr(0, first_delimeter);
				double probability = atof(line.substr(first_delimeter+1, 5).c_str());
				// ROS_INFO("ISR: (%s) Adding to actions_prob_map: %s %f", ros::this_node::getName().c_str(), action_name.c_str(), probability);
				actions_prob_map.insert(std::pair<std::string, double>(action_name, probability));
			}
			else if(line == "-"){
				// ROS_INFO("ISR: (%s) Entered actions part of file", ros::this_node::getName().c_str());
				entering_actions_part = true;
			}
		}

		file.close();
		// printStringDoubleMap(actions_prob_map, "Actions probability map");
	}

	void AdaptablePlanDispatcher::printStringDoubleMap(std::map<std::string,double> m, std::string msg){
		std::stringstream ss;
		for (auto const& pair: m) {
			ss << "\n{" << pair.first.c_str() << ": " << pair.second << "}";
		}
		ROS_INFO("ISR: (%s)\n%s: %s", ros::this_node::getName().c_str(), msg.c_str(), ss.str().c_str());
	}

	void AdaptablePlanDispatcher::registerError(){
		std::ofstream outfile;
		ROS_INFO("ISR: (%s) Registering error", ros::this_node::getName().c_str());
		outfile.open("/home/tomas/ros_ws/src/ROSPlan/src/rosplan/number_errors.txt", std::ios_base::app); // append instead of overwrite
		outfile << "1";
		outfile.close(); 
	}

	/*-----------------*/
	/* action dispatch */
	/*-----------------*/

	/*
	 * Loop through and publish planned actions
	 */
	bool AdaptablePlanDispatcher::dispatchPlan(double missionStartTime, double planStartTime) {

		ROS_INFO("KCL: (%s) Dispatching plan.", ros::this_node::getName().c_str());

		mission_start_time = ros::WallTime::now().toSec();

		ros::Rate loop_rate(10);
		replan_requested = false;
		plan_cancelled = false;

		// initialise machine
		initialise();

		// begin execution
		finished_execution = false;
		state_changed = false;
		bool plan_started = false;

		while (ros::ok() && !finished_execution) {

			// ROS_INFO("ISR: (%s) Beginning while loop", ros::this_node::getName().c_str());

			// loop while dispatch is paused
			while (ros::ok() && dispatch_paused) {
				ros::spinOnce();
				loop_rate.sleep();
			}

			// cancel plan
			if(plan_cancelled) {
				ROS_INFO("KCL: (%s) Plan cancelled.", ros::this_node::getName().c_str());
				break;
			}

			finished_execution = true;
			state_changed = false;

			// printEsterelPlan();

			// printMap(action_dispatched, "action_dispatched");
			// printMap(action_completed, "action_completed");
			// ROS_INFO("ISR: (%s) Going inside for loop", ros::this_node::getName().c_str());
			// for nodes check conditions, and dispatch
			std::vector<rosplan_dispatch_msgs::EsterelPlanNode>::const_iterator ci;

			for(ci = current_plan.nodes.begin(); ci != current_plan.nodes.end(); ci++) {

				rosplan_dispatch_msgs::EsterelPlanNode node = *ci;

				// ROS_INFO("ISR: (%s) -----------------------------", ros::this_node::getName().c_str());
				// ROS_INFO("ISR: (%s) Action id: %d", ros::this_node::getName().c_str(), node.action.action_id);
				// ROS_INFO("ISR: (%s) Full node name: %s", ros::this_node::getName().c_str(), getFullActionName(node).c_str());

				map_node_id[getActionNameWithoutTime(node)] = node.action.action_id;

				// std::string node_type;
				// if(node.node_type == node.ACTION_START)
				// 	node_type = "ACTION_START";
				// else if(node.node_type == node.ACTION_END)
				// 	node_type = "ACTION_END";
				// else if(node.node_type == node.PLAN_START)
				// 	node_type = "PLAN_START";
				// ROS_INFO("ISR: (%s) Node type: %s", ros::this_node::getName().c_str(), node_type.c_str());

				// std::vector<int> edges_in = node.edges_in;
				// std::vector<int> edges_out = node.edges_out;
				// printVectorInts(edges_in, "Edges in");
				// printVectorInts(edges_out, "Edges out");

				// if(node.node_type == rosplan_dispatch_msgs::EsterelPlanNode::ACTION_START){
				// 	ROS_INFO("ISR: (%s) Node is action start: %d", ros::this_node::getName().c_str(), node.node_id);
				// }
				// else if(node.node_type == rosplan_dispatch_msgs::EsterelPlanNode::ACTION_END){
				// 	ROS_INFO("ISR: (%s) Node is action end: %d", ros::this_node::getName().c_str(), node.node_id);
				// }

				// activate plan start edges
				if(node.node_type == rosplan_dispatch_msgs::EsterelPlanNode::PLAN_START && !plan_started) {
					// activate new edges
					std::vector<int>::const_iterator ci = node.edges_in.begin();
					ci = node.edges_out.begin();
					for(; ci != node.edges_out.end(); ci++) {
						edge_active[*ci] = true;
					}

					finished_execution = false;
					// state_changed = true;
					plan_started = true;
				}

				// do not check actions for nodes which are not action nodes
				if(node.node_type != rosplan_dispatch_msgs::EsterelPlanNode::ACTION_START && node.node_type != rosplan_dispatch_msgs::EsterelPlanNode::ACTION_END){
					continue;
				}

				// If at least one node is still executing we are not done yet
				if (action_dispatched[node.action.action_id] && !action_completed[node.action.action_id]) {
					finished_execution = false;
				}

				// check action edges
				bool edges_activate_action = true;
				std::vector<int>::iterator eit = node.edges_in.begin();
				for (; eit != node.edges_in.end(); ++eit) {
					if(!edge_active[(*eit)]){
						edges_activate_action = false;
					}
				}
				if(!edges_activate_action) continue;

				// dispatch new action
				if(node.node_type == rosplan_dispatch_msgs::EsterelPlanNode::ACTION_START && !action_dispatched[node.action.action_id]) {
					// ROS_INFO("ISR: (%s) Trying to dispatch start action", ros::this_node::getName().c_str());

					finished_execution = false;

					// query KMS for condition edges
					bool condition_activate_action = false;
					if(edges_activate_action) {
						condition_activate_action = checkStartPreconditions(node.action);
					}

					if(condition_activate_action) {

						double random_number = (double) rand()/RAND_MAX;
						std::string full_action_name = getFullActionName(node);
						std::string action_name_no_time = getActionNameWithoutTime(node);

						// ROS_INFO("ISR: (%s) Random number: %f", ros::this_node::getName().c_str(), random_number);
						// ROS_INFO("ISR: (%s) Action success probability [%s]: %f", ros::this_node::getName().c_str(),
						// 														  action_name_no_time.c_str(),
						// 														  actions_prob_map[action_name_no_time]);

						// If action failed due to perturbations
						if(random_number>actions_prob_map[action_name_no_time]){
							ROS_INFO("ISR: (%s) Dispatching action start failed due to perturbations [%s]",
                                                                                    ros::this_node::getName().c_str(),
                                                                                    full_action_name.c_str());
							perturbWorldState();
							state_changed = true;

							// Increment action count
							std_srvs::Trigger srv2;
							if(action_counter_client_.call(srv2)){
								// ROS_INFO("ISR: (%s) Successfully increased action count", ros::this_node::getName().c_str());
							}
							else{
								// ROS_INFO("ISR: (%s) Failed to increase action count", ros::this_node::getName().c_str());	
							}

							break;
						}
						else{
							// activate action
							action_received[node.action.action_id] = false;

							// dispatch action start
							ROS_INFO("KCL: (%s) Dispatching action start [%s]", ros::this_node::getName().c_str(), full_action_name.c_str());

							action_dispatch_publisher.publish(node.action);
							actions_executing.push_back(node.node_id);
							state_changed = true;

							// deactivate incoming edges
							std::vector<int>::const_iterator ci = node.edges_in.begin();
							for(; ci != node.edges_in.end(); ci++) {
								edge_active[*ci] = false;
							}

							// activate new edges
							ci = node.edges_out.begin();
							for(; ci != node.edges_out.end(); ci++) {
								edge_active[*ci] = true;
							}

							// Waits for the set time in microseconds
							// Wait so the print of rosplan_interface occurs
							// before the print of the state's perturbation
							usleep(10000);
							perturbWorldState();
							number_actions_dispatched++;

							break;
						}
					}
					else{
						ROS_INFO("ISR: (%s) Action is no longer applicable [%s]",
								ros::this_node::getName().c_str(),
								getFullActionName(node).c_str());
						ROS_INFO("ISR: (%s) Must build a new plan", ros::this_node::getName().c_str());
						state_changed = true;
						break;
						// continue;
					}
				}

				// handle completion of an action
				if(node.node_type == rosplan_dispatch_msgs::EsterelPlanNode::ACTION_END && action_completed[node.action.action_id]) {
					// ROS_INFO("ISR: (%s) Trying to dispatch end action", ros::this_node::getName().c_str());

					// query KMS for condition edges
					bool condition_activate_action = false;
					if(edges_activate_action) {
						condition_activate_action = checkEndPreconditions(node.action);
					}

					if(condition_activate_action) {

						// dispatch action end
						ROS_INFO("KCL: (%s) Dispatching action end [%s]", ros::this_node::getName().c_str(), getFullActionName(node).c_str());

						finished_execution = false;
						state_changed = true;
						actions_executing.push_back(node.node_id);
						action_dispatch_publisher.publish(node.action);

						// deactivate incoming edges
						std::vector<int>::const_iterator ci = node.edges_in.begin();
						for(; ci != node.edges_in.end(); ci++) {
							edge_active[*ci] = false;
						}

						// activate new edges
						ci = node.edges_out.begin();
						for(; ci != node.edges_out.end(); ci++) {
							edge_active[*ci] = true;
						}

						// Waits for the set time in microseconds
						// Wait so the print of rosplan_interface occurs
						// before the print of the state's perturbation
						usleep(10000);
						perturbWorldState();
						number_actions_dispatched++;

						break;
					}
					else{
						ROS_INFO("ISR: (%s) Action is no longer applicable [%s]",
								ros::this_node::getName().c_str(),
								getFullActionName(node).c_str());
						ROS_INFO("ISR: (%s) Must build a new plan", ros::this_node::getName().c_str());
						state_changed = true;
						break;
						// continue;
					}
				}

			} // end loop (action nodes)
			// ROS_INFO("ISR: (%s) Exiting for loop", ros::this_node::getName().c_str());

			ros::spinOnce();
			loop_rate.sleep();

			// if(goalAchieved()){
			// number_actions_dispatched%2 == 0 because the total number
			// of action nodes must be pair, because each action has a start and end
			if(goalAchieved() && (number_actions_dispatched%2 == 0)){
				ROS_INFO("KCL: (%s) Goal is achieved", ros::this_node::getName().c_str());
				finished_execution = true;
			}
			else if(goalAchieved()){
				ROS_INFO("ISR: (%s) Goal is achieved but must finish actions first", ros::this_node::getName().c_str());
                ROS_INFO("KCL: (%s) Calling the alternatives generator.", ros::this_node::getName().c_str());
                rosplan_dispatch_msgs::ExecAlternatives srv;
                srv.request.actions_executing = actions_executing;
                if(!gen_alternatives_client.call(srv)) {
                    ROS_ERROR("KCL: (%s) could not call the generate alternatives service.", ros::this_node::getName().c_str());
                    return false;
                }
                replan_requested = srv.response.replan_needed;
				ROS_INFO("ISR: (%s) Next action: %s", ros::this_node::getName().c_str(), srv.response.next_action.c_str());
				makeOnlyNextActionApplicable(srv.response.next_action);
    			plan_received = false;
		        while (ros::ok() && !plan_received && !replan_requested) {
                    ros::spinOnce();
                    loop_rate.sleep();
                }
                
				ROS_INFO("KCL: (%s) Restarting the dispatch loop.", ros::this_node::getName().c_str());
			}
			else if(state_changed) {
				ROS_INFO("KCL: (%s) Goal is not achieved", ros::this_node::getName().c_str());
                ROS_INFO("KCL: (%s) Calling the alternatives generator.", ros::this_node::getName().c_str());
                rosplan_dispatch_msgs::ExecAlternatives srv;
                srv.request.actions_executing = actions_executing;
				// printVectorInts(actions_executing, "Sending actions executing");
                if(!gen_alternatives_client.call(srv)) {
                    ROS_ERROR("KCL: (%s) could not call the generate alternatives service.", ros::this_node::getName().c_str());
					registerError();
                    return false;
                }
                replan_requested = srv.response.replan_needed;
				// ROS_INFO("ISR: (%s) Next action: %s", ros::this_node::getName().c_str(), srv.response.next_action.c_str());
				makeOnlyNextActionApplicable(srv.response.next_action);
    			plan_received = false;
		        while (ros::ok() && !plan_received && !replan_requested) {
                    ros::spinOnce();
                    loop_rate.sleep();
                }
                
				ROS_INFO("KCL: (%s) Restarting the dispatch loop.", ros::this_node::getName().c_str());
            }

			// cancel dispatch on replan
			if(replan_requested) {
				ROS_INFO("KCL: (%s) Replan requested.", ros::this_node::getName().c_str());
				reset();
				return false;
			}
		}

		ROS_INFO("KCL: (%s) Dispatch complete.", ros::this_node::getName().c_str());

		reset();
		return true;
	}

	void AdaptablePlanDispatcher::initialise() {

		for(std::vector<rosplan_dispatch_msgs::EsterelPlanNode>::const_iterator ci = current_plan.nodes.begin(); ci != current_plan.nodes.end(); ci++) {
			action_dispatched[ci->action.action_id] = false;
			action_received[ci->action.action_id] = false;
			action_completed[ci->action.action_id] = false;
		}

		for(std::vector<rosplan_dispatch_msgs::EsterelPlanEdge>::const_iterator ci = current_plan.edges.begin(); ci != current_plan.edges.end(); ci++) {
			edge_active[ci->edge_id] = false;
		}
	}

	/*------------------*/
	/* general feedback */
	/*------------------*/

	/**
	 * listen to and process actionFeedback topic.
	 */
	void AdaptablePlanDispatcher::feedbackCallback(const rosplan_dispatch_msgs::ActionFeedback::ConstPtr& msg) {

		ROS_INFO("KCL: (%s) Feedback received [%i, %s]", ros::this_node::getName().c_str(), msg->action_id, msg->status.c_str());

		// action enabled
		if(!action_received[msg->action_id] && (0 == msg->status.compare("action enabled"))) {
			action_received[msg->action_id] = true;
		}

		// action completed (successfuly)
		if(!action_completed[msg->action_id] && 0 == msg->status.compare("action achieved")) {

			// check action is part of current plan
			if(!action_received[msg->action_id]) {
				ROS_INFO("KCL: (%s) Action not yet dispatched, ignoring feedback", ros::this_node::getName().c_str());
			}
			action_completed[msg->action_id] = true;
		}

		// action completed (failed)
		if(!action_completed[msg->action_id] && 0 == msg->status.compare("action failed")) {
			replan_requested = true;
			action_completed[msg->action_id] = true;
		}
	}

	/*-------------------*/
	/* Produce DOT graph */
	/*-------------------*/

	void AdaptablePlanDispatcher::printPlan() {

		// output stream
		std::stringstream dest;

		dest << "digraph plan" << " {" << std::endl;

		// nodes
		for(std::vector<rosplan_dispatch_msgs::EsterelPlanNode>::iterator nit = current_plan.nodes.begin(); nit!=current_plan.nodes.end(); nit++) {

			std::stringstream params;
			// do not print parameters for start node
			if(nit->node_type != rosplan_dispatch_msgs::EsterelPlanNode::PLAN_START) {
				// to print action parameters in graph, get parameters from action
				for(auto pit = nit->action.parameters.begin(); pit != nit->action.parameters.end(); pit++) {
					params << pit-> value << ",";
				}
				// replace last character "," with a ")"
				params.seekp(-1, params.cur); params << ')';
				dest <<  nit->node_id << "[ label=\"" << nit->node_id << ". " << nit->name << "\n(" << params.str();
			}
			else {

				dest <<  nit->node_id << "[ label=\"" << nit->node_id << ". " << nit->name;
			}

			switch(nit->node_type) {
			case rosplan_dispatch_msgs::EsterelPlanNode::ACTION_START:
				if(action_received[nit->action.action_id]) {
					dest << "\",style=filled,fillcolor=darkolivegreen,fontcolor=white];" << std::endl;
				} else if(action_dispatched[nit->action.action_id]) {
					dest << "\",style=filled,fillcolor=darkgoldenrod2];" << std::endl;
				} else {
					dest << "\"];" << std::endl;
				}
				break;
			case rosplan_dispatch_msgs::EsterelPlanNode::ACTION_END:
				if(action_completed[nit->action.action_id]) {
					dest << "\",style=filled,fillcolor=darkolivegreen,fontcolor=white];" << std::endl;
				} else if(action_dispatched[nit->action.action_id]) {
					dest << "\",style=filled,fillcolor=darkgoldenrod2];" << std::endl;
				} else {
					dest << "\"];" << std::endl;
				}
				break;
			case rosplan_dispatch_msgs::EsterelPlanNode::PLAN_START:
				dest << "\",style=filled,fillcolor=black,fontcolor=white];" << std::endl;
				break;
			default:
				dest << "\"];" << std::endl;
				break;
			}
		}

		// edges
		for(std::vector<rosplan_dispatch_msgs::EsterelPlanEdge>::iterator eit = current_plan.edges.begin(); eit!=current_plan.edges.end(); eit++) {
			for(int j=0; j<eit->sink_ids.size(); j++) {
			for(int i=0; i<eit->source_ids.size(); i++) {

				dest << "\"" << eit->source_ids[i] << "\"" << " -> \"" << eit->sink_ids[j] << "\"";
				if(eit->duration_upper_bound == std::numeric_limits<double>::max()) {
					dest << " [ label=\"" << eit->edge_id << "[" << eit->duration_lower_bound << ", " << "inf]\"";
				} else {
					dest << " [ label=\"" << eit->edge_id << "[" << eit->duration_lower_bound << ", " << eit->duration_upper_bound << "]\"";
				}

				// decide edge color
				std::string edge_color = "black";

				if(display_edge_type_) {

					// green if conditional edge, red if start to end, blue if interference edge
					if(eit->edge_type == rosplan_dispatch_msgs::EsterelPlanEdge::CONDITION_EDGE){
					edge_color = "green";
					}
					else if(eit->edge_type == rosplan_dispatch_msgs::EsterelPlanEdge::INTERFERENCE_EDGE){
							edge_color = "blue";
					}
					else if(eit->edge_type == rosplan_dispatch_msgs::EsterelPlanEdge::START_END_ACTION_EDGE){
							edge_color = "red";
					}
				}
				else {

					if(edge_active[eit->edge_id]) {
							edge_color = "red";
					}
					else {
							edge_color = "black";
					}
				}

				dest << " , penwidth=2, color=\"" << edge_color << "\"]" << std::endl;

			}};
		}

		dest << "}" << std::endl;

		// publish on topic
		std_msgs::String msg;
		msg.data = dest.str();
		plan_graph_publisher.publish(msg);
	}
} // close namespace

	/*-------------*/
	/* Main method */
	/*-------------*/

	int main(int argc, char **argv) {

		ros::init(argc,argv,"rosplan_esterel_plan_dispatcher");
		ros::NodeHandle nh("~");

		KCL_rosplan::AdaptablePlanDispatcher epd(nh);

		// subscribe to planner output
		std::string planTopic = "complete_plan";
		nh.getParam("plan_topic", planTopic);
		ros::Subscriber plan_sub = nh.subscribe(planTopic, 1, &KCL_rosplan::AdaptablePlanDispatcher::planCallback, &epd);

		std::string feedbackTopic = "action_feedback";
		nh.getParam("action_feedback_topic", feedbackTopic);
		ros::Subscriber feedback_sub = nh.subscribe(feedbackTopic, 1000, &KCL_rosplan::AdaptablePlanDispatcher::feedbackCallback, &epd);

		ROS_INFO("KCL: (%s) Ready to receive", ros::this_node::getName().c_str());
		ros::spin();

		return 0;
	}
