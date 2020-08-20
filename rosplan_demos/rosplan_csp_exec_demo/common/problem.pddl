(define (problem task)
(:domain skip_door)
(:objects
    mbot - robot
    door1 door2 door3 - door
    wp1 wp2 wp3 - waypoint
)
(:init
    (robot_at mbot wp1)

    (connected wp1 wp2)
    (connected wp2 wp3)

