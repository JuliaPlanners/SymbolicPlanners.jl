(define (domain doors-keys-gems)
    (:requirements :fluents :adl :typing)
    (:types key gem - item door - object)
    (:predicates (has ?i - item) (locked ?d - door))
    (:functions (xpos) (ypos) - integer
                (xloc ?o - object) (yloc ?o - object) - integer
                (walls) - bit-matrix)
    (:action pickup
     :parameters (?i - item)
     :precondition (and (not (has ?i)) (= xpos (xloc ?i)) (= ypos (yloc ?i)))
     :effect (and (has ?i) (assign (xloc ?i) -1) (assign (yloc ?i) -1))
    )
    (:action unlock
     :parameters (?k - key ?d - door)
     :precondition (and (has ?k) (locked ?d)
                        (or (and (= xpos (xloc ?d)) (= (- ypos 1) (yloc ?d)))
                            (and (= xpos (xloc ?d)) (= (+ ypos 1) (yloc ?d)))
                            (and (= (- xpos 1) (xloc ?d)) (= ypos (yloc ?d)))
                            (and (= (+ xpos 1) (xloc ?d)) (= ypos (yloc ?d)))))
     :effect (and (not (has ?k)) (not (locked ?d)))
    )
    (:action up
     :precondition
        (and (> ypos 1)
            (= (get-index walls (- ypos 1) xpos) false)
            (not (exists (?d - door)
                (and (locked ?d) (= xpos (xloc ?d)) (= (- ypos 1) (yloc ?d))))))
     :effect (decrease ypos 1)
    )
    (:action down
     :precondition
        (and (< ypos (height walls))
            (= (get-index walls (+ ypos 1) xpos) false)
            (not (exists (?d - door)
                (and (locked ?d) (= xpos (xloc ?d)) (= (+ ypos 1) (yloc ?d))))))
     :effect (increase ypos 1)
    )
    (:action left
     :precondition
        (and (> xpos 1)
            (= (get-index walls ypos (- xpos 1)) false)
            (not (exists (?d - door)
                (and (locked ?d) (= ypos (yloc ?d)) (= (- xpos 1) (xloc ?d))))))
     :effect (decrease xpos 1)
    )
    (:action right
     :precondition
        (and (< xpos (width walls))
            (= (get-index walls ypos (+ xpos 1)) false)
            (not (exists (?d - door)
                (and (locked ?d) (= ypos (yloc ?d)) (= (+ xpos 1) (xloc ?d))))))
     :effect (increase xpos 1)
    )
)
