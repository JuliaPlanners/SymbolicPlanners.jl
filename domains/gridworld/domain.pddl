(define (domain gridworld)
    (:requirements :fluents)
    (:predicates (wall ?x ?y))
    (:functions (xpos) (ypos) (width) (height))
    (:action up
     :precondition (and (< ypos height) (not (wall xpos (+ ypos 1))))
     :effect (increase ypos 1)
    )
    (:action down
     :precondition (and (> ypos 1) (not (wall xpos (- ypos 1))))
     :effect (decrease ypos 1)
    )
    (:action right
     :precondition (and (< xpos width) (not (wall (+ xpos 1) ypos)))
     :effect (increase xpos 1)
    )
    (:action left
     :precondition (and (> xpos 1) (not (wall (- xpos 1) ypos)))
     :effect (decrease xpos 1)
    )
)
