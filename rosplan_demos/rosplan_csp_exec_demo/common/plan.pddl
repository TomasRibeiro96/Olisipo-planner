Number of literals: 41
Constructing lookup tables: [10%] [20%] [30%] [40%] [50%] [60%] [70%] [80%] [90%] [100%]
Post filtering unreachable actions:  [10%] [20%] [30%] [40%] [50%] [60%] [70%] [80%] [90%] [100%]
[01;34mNo analytic limits found, not considering limit effects of goal-only operators[00m
All the ground actions in this problem are compression-safe
Initial heuristic = 8.000
b (7.000 | 3.000)b (6.000 | 8.001)b (5.000 | 8.001)b (4.000 | 23.002)b (3.000 | 30.002)b (2.000 | 58.002)b (1.000 | 71.002);;;; Solution Found
; States evaluated: 42
; Cost: 86.002
; Time 0.02
0.000: (goto_waypoint robot0 wp0 machine1)  [3.000]
0.000: (goto_waypoint robot1 wp0 machine1)  [3.000]
0.000: (goto_waypoint robot2 wp0 wp2)  [5.000]
3.001: (switch_machine_on robot0 machine1)  [5.000]
5.001: (goto_waypoint robot2 wp2 wp3)  [18.000]
8.002: (wait_load_at_machine robot1 robot0 machine1)  [15.000]
23.002: (goto_waypoint robot1 machine1 wp2)  [7.000]
23.002: (goto_waypoint robot2 wp3 machine2)  [13.000]
23.002: (goto_waypoint robot0 machine1 machine2)  [20.000]
30.002: (wait_unload robot1 wp2)  [15.000]
36.002: (switch_machine_on robot2 machine2)  [5.000]
43.002: (wait_load_at_machine robot2 robot0 machine2)  [15.000]
58.002: (goto_waypoint robot2 machine2 wp3)  [13.000]
58.002: (goto_waypoint robot0 machine2 wp3)  [13.000]
71.002: (wait_unload robot2 wp3)  [15.000]
