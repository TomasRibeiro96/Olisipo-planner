(define (domain factory_robot)

(:requirements :strips :typing :fluents :durative-actions :timed-initial-literals)

(:types
	robot
    machine
)

(:predicates
	(robot_at ?r - robot ?m - machine)
    (machine_is_fixed ?m - machine)
)

(:durative-action navigate
	:parameters (?r - robot ?origin ?dest - machine)
	:duration ( = ?duration 1)
	:condition (and
                (at start (robot_at ?r ?origin))
			   )
	:effect (and
				(at start (not (robot_at ?r ?origin)))
				(at end (robot_at ?r ?dest))
		    )
)

(:durative-action fix_machine
	:parameters (?r - robot ?m - machine)
	:duration ( = ?duration 1)
	:condition (and
                (at start (robot_at ?r ?m))
                (over all (robot_at ?r ?m))
                (at end (robot_at ?r ?m))
			   )
	:effect (and
				(at end (machine_is_fixed ?m))
		    )
)


)