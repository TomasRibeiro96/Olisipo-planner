(define (problem task)
(:domain simple_factory_robot)

(:objects
    m1 m2 m3 m4 - machine
)

(:init
    (machine_is_working m1)
    (machine_is_working m2)
    (machine_is_working m3)
    (machine_is_working m4)
)

(:goal (and
            (machine_is_maintained m1)
            (machine_is_maintained m2)
            (machine_is_maintained m3)
            (machine_is_maintained m4)
       )
)

)