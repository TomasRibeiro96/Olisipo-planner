;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;;; 4 Op-blocks world
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

(define (domain BLOCKS)

(:requirements :strips :typing :durative-actions :timed-initial-literals)

(:types block)

(:predicates
	(on ?x - block ?y - block)
	(ontable ?x - block)
	(clear ?x - block)
	(handempty)
	(holding ?x - block)
)

(:durative-action pick-up
	:parameters (?x - block)
	:duration (= ?duration 1)
	:condition (and
		(at start (clear ?x))
		(at start (ontable ?x))
		(at start (handempty))
	)
	:effect(and
		(at end (not (ontable ?x)))
		(at end (not (clear ?x)))
		(at end (not (handempty)))
		(at end (holding ?x))
	)
)

(:durative-action put-down
	:parameters (?x - block)
	:duration (= ?duration 1)
	:condition (and
		(at start (holding ?x))
	)
	:effect(and
		(at end (not (holding ?x)))
		(at end (clear ?x))
		(at end (handempty))
		(at end (ontable ?x))
	)
)

(:durative-action stack
	:parameters (?x - block ?y - block)
	:duration (= ?duration 1)
	:condition (and
		(at start (clear ?y))
		(at start (holding ?x))
	)
	:effect(and
		(at end (clear ?x))
		(at end (not (clear ?y)))
		(at end (handempty))
		(at end (not (holding ?x)))
		(at end (on ?x ?y))
	)
)

(:durative-action unstack
	:parameters (?x - block ?y - block)
	:duration (= ?duration 1)
	:condition (and
		(at start (clear ?x))
		(at start (on ?x ?y))
		(at start  (handempty))
	)
	:effect(and
		(at end (clear ?y))
		(at end (not (clear ?x)))
		(at end (not (handempty)))
		(at end (holding ?x))
		(at end (not (on ?x ?y)))
	)
)
)