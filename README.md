# Olisipo Algorithm

Forked from the ROSPlan repository.
The algorithm is inside [rosplan_planning_system/src/PlanDispatch/CSPExecGenerator.cpp](https://github.com/TomasRibeiro96/Olisipo-planner/blob/8c811522c11bb0bdae573d501a872c419d512440/rosplan_planning_system/src/PlanDispatch/CSPExecGenerator.cpp).

The batch file to run the experiments is in [rosplan_demos/rosplan_csp_exec_demo/scripts/automated_multiple_experiment_run-tomas.sh](https://github.com/TomasRibeiro96/Olisipo-planner/blob/c83f35a0d284f996ed998147c71477d1ffd36dce/rosplan_demos/rosplan_csp_exec_demo/scripts/automated_multiple_experiment_run-tomas.sh).

The script to build the DBN is inside [rosplan_demos/rosplan_csp_exec_demo/scripts/calculate_plan_prob.py](https://github.com/TomasRibeiro96/Olisipo-planner/blob/c83f35a0d284f996ed998147c71477d1ffd36dce/rosplan_demos/rosplan_csp_exec_demo/scripts/calculate_plan_prob.py).


# Rosplan
The main ROSPlan website and documentation is available [here](http://kcl-planning.github.io/ROSPlan).

The ROSPlan framework provides a generic method for task planning in a ROS system. ROSPlan encapsulates both planning and dispatch. It provides with a simple interface, and already includes interfaces to common ROS libraries.

## Installation

ROSPlan supports kinetic and melodic ROS distributions and is developed under Ubuntu 16.04 / 18.04, being linux Ubuntu 18.04 + melodic the recommended setup.

Install dependencies:
```sh
sudo apt install flex bison freeglut3-dev libbdd-dev python-catkin-tools ros-$ROS_DISTRO-tf2-bullet
```

Compile everything:
```sh
catkin build
```
