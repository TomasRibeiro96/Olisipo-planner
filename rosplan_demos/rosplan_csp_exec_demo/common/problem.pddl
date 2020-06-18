(define (problem task)
(:domain robot_delivery)
(:objects
    wp0 wp1 wp2 wp3 - waypoint
    robot0 robot1 robot2 - robot
    machine0 machine1 machine2 - machine
)
(:init
    (robot_at robot0 wp0)
    (robot_at robot1 wp0)
    (robot_at robot2 wp0)

    (undocked robot0)
    (undocked robot1)
    (undocked robot2)


    (localised robot0)
    (localised robot1)
    (localised robot2)



    (nocarrying_order robot0)
    (nocarrying_order robot1)
    (nocarrying_order robot2)

