(define (domain factory_robot)

(:requirements :strips :typing :fluents :durative-actions :timed-initial-literals)

(:types
    machine
)

(:predicates
    (machine_is_maintained ?m - machine)
	(machine_is_working ?m - machine)
	(robot_at ?m - machine)
)

(:durative-action maintain_machine
	:parameters (?m - machine)
	:duration ( = ?duration 1)
	:condition (and
				(at start (machine_is_working ?m))
				(at start (robot_at ?m))
				(over all (machine_is_working ?m))
				(over all (robot_at ?m))
				(at end (machine_is_working ?m))
				(at end (robot_at ?m))
			   )
	:effect (and
				(at end (machine_is_maintained ?m))
		    )
)

(:durative-action go_to_machine
	:parameters (?o ?d - machine)
	:duration ( = ?duration 1)
	:condition (and
				(at start (robot_at ?o))
			   )
	:effect (and
				(at start (not (robot_at ?o)))
				(at end (robot_at ?d))
		    )
)

)