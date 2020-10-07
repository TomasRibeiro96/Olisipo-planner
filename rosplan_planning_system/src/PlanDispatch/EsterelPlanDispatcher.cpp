#include "rosplan_planning_system/PlanDispatch/EsterelPlanDispatcher.h"
#include <fstream>


namespace KCL_rosplan {

    /*-------------*/
    /* constructor */
    /*-------------*/

    EsterelPlanDispatcher::EsterelPlanDispatcher(ros::NodeHandle& nh): display_edge_type_(false),
        PlanDispatcher(nh)  {

        node_handle = &nh;

		ros::NodeHandle nh_;
		perturb_client_ = nh_.serviceClient<rosplan_knowledge_msgs::PerturbStateService>("perturb_state");

        // robust experiment parameters
        timeout_actions = false;
        action_timeout_fraction = 0;
        // Tomas: Found out that this line below is not
        // actually getting the parameter
        nh.getParam("timeout_actions", timeout_actions);
        nh.getParam("action_timeout_fraction", action_timeout_fraction);
        
        std::string planTopic = "complete_plan";
        nh.getParam("plan_topic", planTopic);

        std::string plan_graph_topic = "plan_graph";
        nh.getParam("plan_graph_topic", plan_graph_topic);
        plan_graph_publisher = node_handle->advertise<std_msgs::String>(plan_graph_topic, 1000, true);

        // display edge type with colors (conditional edge, interference edge, etc)
        nh.param("display_edge_type", display_edge_type_, false);

        std::string probabilities_file;
		nh.getParam("probabilities_file", probabilities_file);
		fillProbabilitiesMap(probabilities_file);

		action_counter_client_ = nh_.serviceClient<std_srvs::Trigger>("increment_action_count");

        get_action_count_client_ = nh_.serviceClient<std_srvs::Trigger>("action_count");

        reset();
    }

    EsterelPlanDispatcher::~EsterelPlanDispatcher()
    {

    }

    void EsterelPlanDispatcher::reset() {

        // preempt currently executing nodes
        for(std::vector<rosplan_dispatch_msgs::EsterelPlanNode>::const_iterator ci = current_plan.nodes.begin(); ci != current_plan.nodes.end(); ci++) {
            rosplan_dispatch_msgs::EsterelPlanNode node = *ci;

            // dispatch new action
            if(node.node_type == rosplan_dispatch_msgs::EsterelPlanNode::ACTION_START && action_dispatched[node.action.action_id] && !action_completed[node.action.action_id]) {

                // try to preempt action
                // ROS_INFO("KCL: (%s) Preempting action [%i, %s]",
                //         ros::this_node::getName().c_str(),
                //         node.action.action_id,
                //         node.action.name.c_str());

                node.action.name = "cancel_action";
                action_dispatch_publisher.publish(node.action);
                ros::spinOnce();
            }
        }

        PlanDispatcher::reset();
        finished_execution = true;
    }

    /*-------------------*/
    /* Plan subscription */
    /*-------------------*/

    
        void EsterelPlanDispatcher::planCallback(const rosplan_dispatch_msgs::EsterelPlan plan) {
       
                if(finished_execution) {
                        // ROS_INFO("KCL: (%s) Plan received.", ros::this_node::getName().c_str());
                        plan_received = true;
                        mission_start_time = ros::Time::now().toSec();
                        current_plan = plan;
                        printPlan();
                } else {
                        ROS_INFO("KCL: (%s) Plan received, but current execution not yet finished.", ros::this_node::getName().c_str());
                }
        }

    /*-----------------*/
    /* action dispatch */
    /*-----------------*/

    void EsterelPlanDispatcher::perturbWorldState(){
        //// Perturb state
        rosplan_knowledge_msgs::PerturbStateService srv;
        // ROS_INFO("ISR: (%s) Perturbing world state", ros::this_node::getName().c_str());
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

	std::string EsterelPlanDispatcher::getFullActionName(rosplan_dispatch_msgs::EsterelPlanNode node){
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

	std::string EsterelPlanDispatcher::getActionNameWithoutTime(rosplan_dispatch_msgs::EsterelPlanNode node){
		std::stringstream ss;

		ss << node.action.name;

		std::vector<diagnostic_msgs::KeyValue, std::allocator<diagnostic_msgs::KeyValue>> parameters = node.action.parameters;
		for(diagnostic_msgs::KeyValue param: parameters){
			ss << "#";
			ss << param.value;
		}

		return ss.str();
	}

	void EsterelPlanDispatcher::fillProbabilitiesMap(std::string probabilities_file){
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

	void EsterelPlanDispatcher::printStringDoubleMap(std::map<std::string,double> m, std::string msg){
		std::stringstream ss;
		for (auto const& pair: m) {
			ss << "\n{" << pair.first.c_str() << ": " << pair.second << "}";
		}
		ROS_INFO("ISR: (%s)\n%s: %s", ros::this_node::getName().c_str(), msg.c_str(), ss.str().c_str());
	}



    /*
     * Loop through and publish planned actions
     */
    bool EsterelPlanDispatcher::dispatchPlan(double missionStartTime, double planStartTime) {

        // ROS_INFO("KCL: (%s) Dispatching plan.", ros::this_node::getName().c_str());

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

            // for each node check completion, conditions, and dispatch
            for(std::vector<rosplan_dispatch_msgs::EsterelPlanNode>::const_iterator ci = current_plan.nodes.begin(); ci != current_plan.nodes.end(); ci++) {
                //the main loop
                rosplan_dispatch_msgs::EsterelPlanNode node = *ci;

                // activate plan start edges
                if(node.node_type == rosplan_dispatch_msgs::EsterelPlanNode::PLAN_START && !plan_started) {

                        // record the time for the PLAN_START node
                        double NOW = ros::Time::now().toSec();    
                        node_real_dispatch_time.insert (std::pair<int,double>(node.node_id, NOW)); 

                        // activate new edges
                        std::vector<int>::const_iterator ci = node.edges_in.begin();
                        ci = node.edges_out.begin();
                        for(; ci != node.edges_out.end(); ci++) {
                                edge_active[*ci] = true;
                        }
                        finished_execution = false;
                        state_changed = true;
                        plan_started = true;
                }

                                 
                // do not check actions for nodes which are not action nodes
                if(node.node_type != rosplan_dispatch_msgs::EsterelPlanNode::ACTION_START && node.node_type != rosplan_dispatch_msgs::EsterelPlanNode::ACTION_END)
                    continue;
                                
                // If at least one node is still executing we are not done yet
                if (action_dispatched[node.action.action_id] && !action_completed[node.action.action_id]) {
                    finished_execution = false;
                }

                // check action edges
                bool edges_activate_action = true;
                std::vector<int>::iterator eit = node.edges_in.begin();
                for (; eit != node.edges_in.end(); ++eit) {
                    if(!edge_active[(*eit)]) {
                        edges_activate_action = false;
                        break;
                    }
                }
                if(!edges_activate_action) continue;

                // handle completion of an action
                if(node.node_type == rosplan_dispatch_msgs::EsterelPlanNode::ACTION_END && action_completed[node.action.action_id]) {

                    // query KMS for condition edges
                    bool condition_activate_action = false;
                    if(edges_activate_action) {
                        condition_activate_action = checkEndPreconditions(node.action);
                    }

                    // the state is unexpected
                    if(!condition_activate_action && timeout_actions) {
                        replan_requested = true;
                    }

                    if(condition_activate_action) {
                        std::stringstream params_ss;
                        std::vector<diagnostic_msgs::KeyValue, std::allocator<diagnostic_msgs::KeyValue>> action_params = node.action.parameters;

                        for(diagnostic_msgs::KeyValue param: action_params){
                            params_ss << param.value;
                            params_ss << " ";
                        }

                        // dispatch action end
                        // ROS_INFO("KCL: (%s) Dispatching action end [%s %s]",
                        //         ros::this_node::getName().c_str(),
                        //         node.action.name.c_str(),
                        //         params_ss.str().c_str());

                        finished_execution = false;
                        state_changed = true;
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
                    }
                    else{
						// ROS_INFO("ISR: (%s) Action is no longer applicable [%s]",
						// 		ros::this_node::getName().c_str(),
						// 		getFullActionName(node).c_str());
						// ROS_INFO("ISR: (%s) Must build a new plan", ros::this_node::getName().c_str());
                        replan_requested = true;
                        break;
                    }
                }

                if(timeout_actions) {

                    // check time bounds on edges
                    bool times_activate_action = true;
                    eit = node.edges_in.begin();
                    for (; eit != node.edges_in.end(); ++eit) {

                        rosplan_dispatch_msgs::EsterelPlanEdge edge = current_plan.edges[*eit];

                        //define a minimum and maximum dispatch time for each edge
                        float minimum_dispatch_time = node_real_dispatch_time[edge.source_ids[0]] + edge.duration_lower_bound - planStartTime;
                        float maximum_dispatch_time = node_real_dispatch_time[edge.source_ids[0]] + edge.duration_upper_bound - planStartTime;

                        // widen bounds based on parameter (default by 0)
                        minimum_dispatch_time = minimum_dispatch_time - action_timeout_fraction*minimum_dispatch_time;
                        maximum_dispatch_time = maximum_dispatch_time + action_timeout_fraction*maximum_dispatch_time;

                        // check the current time with the lower bound
                         double NOW = ros::Time::now().toSec();
                         if (NOW - planStartTime < minimum_dispatch_time) { 
                            
                            times_activate_action = false;
                            finished_execution = false;
                            break;
                        }

                        // check the current time with the upper bound
                        if (NOW - planStartTime > maximum_dispatch_time) {
                            // don't check deadlines for actions that have actually completed.
                            if(node.node_type != rosplan_dispatch_msgs::EsterelPlanNode::ACTION_END || !action_completed[node.action.action_id]) {
                                replan_requested =  true;
                                times_activate_action = false;
                                ROS_INFO("KCL: (%s) Deadline passed, %s: %f > %f.", ros::this_node::getName().c_str(), node.name.c_str(), NOW-planStartTime, maximum_dispatch_time);
                            }
                        }
                    }

                    // the lower bound of the action is not yet reached
                    if(!times_activate_action) continue;
                }

                // dispatch new action
                if(node.node_type == rosplan_dispatch_msgs::EsterelPlanNode::ACTION_START && !action_dispatched[node.action.action_id]) {

                    finished_execution = false;

                    // query KMS for condition edges
                    bool condition_activate_action = false;
                    if(edges_activate_action) {
                        condition_activate_action = checkStartPreconditions(node.action);
                    }

                    // the state is unexpected
                    if(!condition_activate_action && timeout_actions) {
                        replan_requested = true;
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
							// ROS_INFO("ISR: (%s) Dispatching action start failed due to perturbations [%s]",
                            //                                                         ros::this_node::getName().c_str(),
                            //                                                         full_action_name.c_str());
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
                            action_dispatched[node.action.action_id] = true;
                            action_received[node.action.action_id] = false;
                            action_completed[node.action.action_id] = false;

                            std::stringstream params_ss;
                            std::vector<diagnostic_msgs::KeyValue, std::allocator<diagnostic_msgs::KeyValue>> action_params = node.action.parameters;

                            for(diagnostic_msgs::KeyValue param: action_params){
                                params_ss << param.value;
                                params_ss << " ";
                            }

                            // dispatch action
                            // ROS_INFO("KCL: (%s) Dispatching action start [%s %s]",
                            //         ros::this_node::getName().c_str(),
                            //         node.action.name.c_str(),
                            //         params_ss.str().c_str());

                            action_dispatch_publisher.publish(node.action);
                            
                            // record the dispatch time for action start node
                            double NOW = ros::Time::now().toSec();    
                            node_real_dispatch_time.insert (std::pair<int,double>(node.node_id, NOW));

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
                        }

                        perturbWorldState();
                    }
                    else{
						// ROS_INFO("ISR: (%s) Action is no longer applicable [%s]",
						// 		ros::this_node::getName().c_str(),
						// 		getFullActionName(node).c_str());
						// ROS_INFO("ISR: (%s) Must build a new plan", ros::this_node::getName().c_str());
                        replan_requested = true;
                        break;
                    }
                }
            
                // Get action count
                // std_srvs::Trigger srv3;
                // if(get_action_count_client_.call(srv3)){
                //     ROS_INFO("ISR: (%s) Current action count: %s", ros::this_node::getName().c_str(), srv3.response.message.c_str());
                // }
                // else{
                //     ROS_INFO("ISR: (%s) Failed to get action count", ros::this_node::getName().c_str());	
                // }

            } // end loop (action nodes)

            ros::spinOnce();
            loop_rate.sleep();

            if(state_changed) {
                printPlan();
            }

            // cancel dispatch on replan
            if(replan_requested) {
                // ROS_INFO("KCL: (%s) Replan requested.", ros::this_node::getName().c_str());
                reset();
                return false;
            }
        }

        // ROS_INFO("KCL: (%s) Dispatch complete.", ros::this_node::getName().c_str());

        // reset();
        return true;
    }

    void EsterelPlanDispatcher::initialise() {

        node_real_dispatch_time.clear();

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
    void EsterelPlanDispatcher::feedbackCallback(const rosplan_dispatch_msgs::ActionFeedback::ConstPtr& msg) {

        // ROS_INFO("KCL: (%s) Feedback received [%i, %s]", ros::this_node::getName().c_str(), msg->action_id, msg->status.c_str());

        // action enabled
        if(!action_received[msg->action_id] && (0 == msg->status.compare("action enabled"))) {
            action_received[msg->action_id] = true;
            state_changed = true;
        }

        // action completed (successfuly)
        if(!action_completed[msg->action_id] && 0 == msg->status.compare("action achieved")) {

            // check action is part of current plan
            if(!action_received[msg->action_id]) {
                ROS_WARN("KCL: (%s) Action not yet dispatched, ignoring feedback", ros::this_node::getName().c_str());
            } else {
                for(std::vector<rosplan_dispatch_msgs::EsterelPlanNode>::const_iterator ci = current_plan.nodes.begin(); ci != current_plan.nodes.end(); ci++) {
                    rosplan_dispatch_msgs::EsterelPlanNode node = *ci;
                    if(node.action.action_id == msg->action_id && node.node_type == rosplan_dispatch_msgs::EsterelPlanNode::ACTION_END){
                        // record the time for the end action node
                        double NOW = ros::Time::now().toSec();    
                        node_real_dispatch_time.insert (std::pair<int,double>(node.node_id, NOW)); 
                    }
                }
                action_completed[msg->action_id] = true;
                state_changed = true;
            }
        }

        // action completed (failed)
        if(!action_completed[msg->action_id] && 0 == msg->status.compare("action failed")) {

            // check action is part of current plan
            if(!action_received[msg->action_id]) {
                ROS_WARN("KCL: (%s) Action not yet dispatched, ignoring feedback", ros::this_node::getName().c_str());
            } else {
                replan_requested = true;
                state_changed = true;
                action_completed[msg->action_id] = true;
            }
        }
    }

    /*-------------------*/
    /* Produce DOT graph */
    /*-------------------*/

    bool EsterelPlanDispatcher::printPlan() {

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
                dest <<  nit->node_id << "[ label=\"" << nit->name << "\n(" << params.str();
            }
            else {

                dest <<  nit->node_id << "[ label=\"" << nit->name;
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
                    dest << " [ label=\"[" << eit->duration_lower_bound << ", " << "inf]\"";
                } else {
                    dest << " [ label=\"[" << eit->duration_lower_bound << ", " << eit->duration_upper_bound << "]\"";
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

        KCL_rosplan::EsterelPlanDispatcher epd(nh);

        // subscribe to planner output
        std::string planTopic = "complete_plan";
                //parisa:robust_plan
                //std::string planTopic = "robust_plan";
        nh.getParam("plan_topic", planTopic);
        ros::Subscriber plan_sub = nh.subscribe(planTopic, 1, &KCL_rosplan::EsterelPlanDispatcher::planCallback, &epd);

        std::string feedbackTopic = "action_feedback";
        nh.getParam("action_feedback_topic", feedbackTopic);
        ros::Subscriber feedback_sub = nh.subscribe(feedbackTopic, 1000, &KCL_rosplan::EsterelPlanDispatcher::feedbackCallback, &epd);

        // ROS_INFO("KCL: (%s) Ready to receive", ros::this_node::getName().c_str());
        ros::spin();

        return 0;
    }
