(define (domain fruit_transport)

(:requirements :strips :typing :fluents :durative-actions :timed-initial-literals)

(:types
	robot
	door
	waypoint
    fruit
)

(:predicates

	(robot_at ?r - robot ?wp - waypoint)
	(connected ?wp1 ?wp2 - waypoint)
	(door_at ?d - door ?wp - waypoint)
	(door_is_open ?d - door)
    (fruit_is_good ?f - fruit)
    (fruit_on ?f - fruit ?wp - waypoint)
    (handempty)
    (holding ?f - fruit)
)

(:durative-action navigate
	:parameters (?r - robot ?origin ?dest - waypoint ?dorigin ?ddest - door)
	:duration ( = ?duration 1)
	:condition (and
					(at start (robot_at ?r ?origin))
					(at start (connected ?origin ?dest))
					; Added these at start below
					(at start (door_at ?dorigin ?origin))
				    (at start (door_at ?ddest ?dest))
					(at start (door_is_open ?dorigin))

					(over all (door_at ?dorigin ?origin))
				    (over all (door_at ?ddest ?dest))

					(over all (door_is_open ?dorigin))
			   )
	:effect (and
				(at start (not (robot_at ?r ?origin)))
				(at end (robot_at ?r ?dest))
		    )
)

(:durative-action open_door
	:parameters (?r - robot ?wp - waypoint ?door - door)
	:duration ( = ?duration 1)
	:condition (and
					(at start (robot_at ?r ?wp))
					; Added this at start below
					(at start (door_at ?door ?wp))
					
					(over all (door_at ?door ?wp))
			   )
	:effect (and
				(at end (door_is_open ?door))
		    )
)

(:durative-action pick_up
	:parameters (?r - robot ?wp - waypoint ?f - fruit)
	:duration ( = ?duration 1)
	:condition (and
                (at start (fruit_on ?f ?wp))
                (at start (fruit_is_good ?f))
                (at start (handempty))
                (at start (robot_at ?r ?wp))

                (over all (fruit_is_good ?f))
                (over all (robot_at ?r ?wp))
			   )
	:effect (and
				(at end (not (fruit_on ?f ?wp)))
                (at_end (not (handempty)))
                (at end (holding ?f))
		    )
)

(:durative-action put_down
	:parameters (?r - robot ?wp - waypoint ?f - fruit)
	:duration ( = ?duration 1)
	:condition (and
                (at start (fruit_is_good ?f))
                (at start (holding ?f))
                (at start not (handempty))
                (at start (robot_at ?r ?wp))

                (over all (fruit_is_good ?f))
                (over all (robot_at ?r ?wp))
			   )
	:effect (and
				(at end (fruit_on ?f ?wp))
                (at_end (handempty))
                (at end not (holding ?f))
		    )
)

)
