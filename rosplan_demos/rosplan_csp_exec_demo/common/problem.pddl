(define (problem task)
(:domain factory_robot)
(:objects
    mbot - robot
    m1 m2 m3 - machine
)
(:init
    (robot_at mbot m1)

    (machine_is_fixed m1)

)
(:goal (and
    (machine_is_fixed m1)
    (machine_is_fixed m2)
    (machine_is_fixed m3)
))
)
