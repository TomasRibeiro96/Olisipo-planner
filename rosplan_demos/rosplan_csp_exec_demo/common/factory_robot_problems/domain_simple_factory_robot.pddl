(define (domain factory_robot)

(:requirements :strips :typing :fluents :durative-actions :timed-initial-literals)

(:types
    machine
)

(:predicates
    (machine_is_maintained ?m - machine)
	(machine_is_working ?m - machine)
)

(:durative-action go_maintain_machine
	:parameters (?m - machine)
	:duration ( = ?duration 1)
	:condition (and
				(at start (machine_is_working ?m))
				(over all (machine_is_working ?m))
				(at end (machine_is_working ?m))
			   )
	:effect (and
				(at end (machine_is_maintained ?m))
		    )
)

)