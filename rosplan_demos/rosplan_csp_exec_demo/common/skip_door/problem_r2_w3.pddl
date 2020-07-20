(define (problem task)
(:domain skip_door)
(:objects
    wp1 wp2 wp3 - waypoint
    robot1 robot2 - robot
    door1 door2 door3 - door
)

(:init
    (robot_at robot1 wp1)
    (robot_at robot2 wp1)

    (connected wp1 wp2)
    (connected wp2 wp3)

    (door_at door1 wp1)
    (door_at door2 wp2)
    (door_at door3 wp3)

    (door_is_open door1)
)

(:goal (and
            (robot_at robot1 wp3)
            (robot_at robot2 wp3)
       )
)

)
