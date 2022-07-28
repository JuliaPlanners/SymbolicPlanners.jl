; implementation of the https://en.wikipedia.org/wiki/Wolf,_goat_and_cabbage_problem

(define (domain cabbage_domain)

(:requirements :equality :constraints :typing)

(:types
    pos
)

(:predicates ;todo: define predicates here
    (cabbage-at ?p - pos)
    (goat-at ?p - pos)
    (wolf-at ?p - pos)
    (boat-at ?p - pos)
)

(:action ship-cabbage
    :parameters (?v ?w - pos)
    :precondition (and (cabbage-at ?v) (boat-at ?v))
    :effect (and (cabbage-at ?w) (boat-at ?w) (not (cabbage-at ?v)) (not (boat-at ?v)))
)

(:action ship-goat
    :parameters (?v ?w - pos)
    :precondition (and (goat-at ?v) (boat-at ?v))
    :effect (and (goat-at ?w) (boat-at ?w) (not (goat-at ?v)) (not (boat-at ?v)))
)

(:action ship-wolf
    :parameters (?v ?w - pos)
    :precondition (and (wolf-at ?v) (boat-at ?v))
    :effect (and (wolf-at ?w) (boat-at ?w) (not (wolf-at ?v)) (not (boat-at ?v)))
)

(:action ship-self
    :parameters (?v ?w - pos)
    :precondition (and (boat-at ?v))
    :effect (and (boat-at ?w) (not (boat-at ?v)))
)

)