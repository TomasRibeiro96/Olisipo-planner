(define (problem task)
(:domain fruit_transport)

(:objects
    wp1 wp2 wp3 - waypoint
    mbot - robot
    door1 door2 door3 - door
    f1 f2 - fruit
)

(:init
    (robot_at mbot wp1)

    (connected wp1 wp2)
    (connected wp2 wp3)

    (door_at door1 wp1)
    (door_at door2 wp2)
    (door_at door3 wp3)

    (door_is_open door1)

    (fruit_is_good f1)
    (fruit_is_good f2)

    (fruit_on f1 wp1)
    (fruit_on f2 wp2)

    (handempty)
)

(:goal (and
            (fruit_on f1 wp3)
            (fruit_on f1 wp3)
       )
)

)
