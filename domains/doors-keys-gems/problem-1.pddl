;; ASCII ;;
; W: wall, D: door, k: key, g: gem, G: goal-gem, s: start, .: empty
; sWG
; kW.
; .D.
(define (problem doors-keys-gems-1)
  (:domain doors-keys-gems)
  (:objects door1 - door key1 - key gem1 - gem)
  (:init (= (walls) (new-bit-matrix false 3 3))
         (= (walls) (set-index walls true 1 2))
         (= (xloc gem1) (3))
         (= (yloc gem1) (1))
         (= (xloc key1) (1))
         (= (yloc key1) (2))
         (= (walls) (set-index walls true 2 2))
         (= (xloc door1) (2))
         (= (yloc door1) (3))
         (locked door1)
         (= (xpos) (1))
         (= (ypos) (1)))
  (:goal (has gem1))
)
