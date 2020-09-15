(define (problem task)
(:domain factory_robot)

(:objects
    m1 m2 m3 - machine
    mbot - robot
)

(:init
    ; (robot_at mbot m1)
    (machine_is_working m1)
    (machine_is_working m2)
    (machine_is_working m3)
)

(:goal (and
            (machine_is_fixed m1)
            (machine_is_fixed m2)
            (machine_is_fixed m3)
       )
)

)