(define (problem task)
(:domain skip_door)
(:objects
    wp1 wp2 wp3 wp4 wp5 - waypoint
    mbot - robot
    door1 door2 door3 door4 door5 - door
)

(:init
    (robot_at mbot wp1)

    (connected wp1 wp2)
    (connected wp2 wp3)
    (connected wp3 wp4)
    (connected wp4 wp5)

    (door_at door1 wp1)
    (door_at door2 wp2)
    (door_at door3 wp3)
    (door_at door4 wp4)
    (door_at door5 wp5)

    (door_is_open door1)
)

(:goal (and
            (robot_at mbot wp5)
       )
)

)
