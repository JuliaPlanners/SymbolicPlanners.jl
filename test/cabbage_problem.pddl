(define (problem cabbage_problem) (:domain cabbage_domain)
(:objects 
    left right - pos
)

(:init
    (cabbage-at left)
    (goat-at left)
    (wolf-at left)
    (boat-at left)
)

(:goal (and
    (cabbage-at right)
    (goat-at right)
    (wolf-at right)
    (boat-at right)
))
(:constraints
    (and (forall (?p - pos) (imply (and (cabbage-at ?p) (goat-at ?p)) (boat-at ?p)))
    (forall (?p - pos) (imply (and (wolf-at ?p) (goat-at ?p)) (boat-at ?p))))
)

)
