(define (domain blocks)
  (:requirements :strips :typing)
  (:types block)
  (:predicates (on ?x ?y - block)
	       (on-table ?x - block)
	       (clear ?x - block)
	       (arm-empty)
	       (holding ?x - block)
	       )

  (:action pick-up
	     :parameters (?x - block)
	     :precondition (and (clear ?x) (on-table ?x) (arm-empty))
	     :effect
	     (and (not (on-table ?x))
		   (not (clear ?x))
		   (not (arm-empty))
		   (holding ?x)))

  (:action put-down
	     :parameters (?x - block)
	     :precondition (holding ?x)
	     :effect
	     (and (not (holding ?x))
		   (clear ?x)
		   (arm-empty)
		   (on-table ?x)))
  (:action stack
	     :parameters (?x ?y - block)
	     :precondition (and (holding ?x) (clear ?y))
	     :effect
	     (and (not (holding ?x))
		   (not (clear ?y))
		   (clear ?x)
		   (arm-empty)
		   (on ?x ?y)))
  (:action unstack
	     :parameters (?x ?y - block)
	     :precondition (and (on ?x ?y) (clear ?x) (arm-empty))
	     :effect
	     (and (holding ?x)
		   (clear ?y)
		   (not (clear ?x))
		   (not (arm-empty))
		   (not (on ?x ?y)))))
