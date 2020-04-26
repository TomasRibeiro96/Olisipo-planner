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



    (delivery_destination wp3)
    (delivery_destination wp2)


    (machine_off machine0)
    (machine_off machine1)
    (machine_off machine2)

    (= (distance wp0 wp1) 30)
    (= (distance wp1 wp0) 30)
    (= (distance wp0 wp2) 5)
    (= (distance wp2 wp0) 5)
    (= (distance wp0 wp3) 16)
    (= (distance wp3 wp0) 16)
    (= (distance wp1 wp2) 26)
    (= (distance wp2 wp1) 26)
    (= (distance wp1 wp3) 21)
    (= (distance wp3 wp1) 21)
    (= (distance wp2 wp3) 18)
    (= (distance wp3 wp2) 18)
    (= (distance wp0 machine0) 18)
    (= (distance machine0 wp0) 18)
    (= (distance wp0 machine1) 3)
    (= (distance machine1 wp0) 3)
    (= (distance wp0 machine2) 18)
    (= (distance machine2 wp0) 18)
    (= (distance wp1 machine0) 13)
    (= (distance machine0 wp1) 13)
    (= (distance wp1 machine1) 32)
    (= (distance machine1 wp1) 32)
    (= (distance wp1 machine2) 13)
    (= (distance machine2 wp1) 13)
    (= (distance wp2 machine0) 16)
    (= (distance machine0 wp2) 16)
    (= (distance wp2 machine1) 7)
    (= (distance machine1 wp2) 7)
    (= (distance wp2 machine2) 14)
    (= (distance machine2 wp2) 14)
    (= (distance wp3 machine0) 33)
    (= (distance machine0 wp3) 33)
    (= (distance wp3 machine1) 16)
    (= (distance machine1 wp3) 16)
    (= (distance wp3 machine2) 13)
    (= (distance machine2 wp3) 13)
    (= (distance machine0 machine1) 20)
    (= (distance machine1 machine0) 20)
    (= (distance machine0 machine2) 21)
    (= (distance machine2 machine0) 21)
    (= (distance machine1 machine2) 20)
    (= (distance machine2 machine1) 20)

)
(:goal (and
    (order_delivered wp3)
    (order_delivered wp2)
))
)
