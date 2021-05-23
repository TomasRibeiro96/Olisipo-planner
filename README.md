# Olisipo Algorithm

Forked from the ROSPlan repository.
The algorithm is inside _rosplan_planning_system/src/PlanDispatch/CSPExecGenerator.cpp_.

The batch file to run the experiments is in _rosplan_demos/rosplan_csp_exec_demo/scripts/automated_multiple_experiment_run-tomas.sh_.

The script to build the DBN is inside _rosplan_demos/rosplan_csp_exec_demo/scripts/calculate_plan_prob.py_.


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
