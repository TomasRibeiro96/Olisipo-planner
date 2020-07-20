(define (problem task)
(:domain skip_door)
(:objects
    wp1 wp2 wp3 wp4 wp5 wp6 wp7 wp8 wp9 wp10 wp11 wp12 wp13 wp14 wp15 - waypoint
    mbot - robot
    door1 door2 door3 door4 door5 door6 door7 door8 door9 door10 door11 door12 door13 door14 door15 - door
)

(:init
    (robot_at mbot wp1)

    (connected wp1 wp2)
    (connected wp2 wp3)
    (connected wp3 wp4)
    (connected wp4 wp5)
    (connected wp5 wp6)
    (connected wp6 wp7)
    (connected wp7 wp8)
    (connected wp8 wp9)
    (connected wp9 wp10)
    (connected wp10 wp11)
    (connected wp11 wp12)
    (connected wp12 wp13)
    (connected wp13 wp14)
    (connected wp14 wp15)

    (door_at door1 wp1)
    (door_at door2 wp2)
    (door_at door3 wp3)
    (door_at door4 wp4)
    (door_at door5 wp5)
    (door_at door6 wp6)
    (door_at door7 wp7)
    (door_at door8 wp8)
    (door_at door9 wp9)
    (door_at door10 wp10)
    (door_at door11 wp11)
    (door_at door12 wp12)
    (door_at door13 wp13)
    (door_at door14 wp14)
    (door_at door15 wp15)

    (door_is_open door1)
)

(:goal (and
            (robot_at mbot wp15)
       )
)

)
