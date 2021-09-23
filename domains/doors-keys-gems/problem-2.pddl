;; ASCII ;;
; W: wall, D: door, k: key, g: gem, G: goal-gem, s: start, .: empty
; ...D.
; .W.Wg
; sWkW.
; .W.WG
; ...D.
(define (problem doors-keys-gems-2)
  (:domain doors-keys-gems)
  (:objects door1 door2 - door key1 - key gem1 gem2 - gem)
  (:init (= (walls) (new-bit-matrix false 5 5))
         (= (xloc door1) (4))
         (= (yloc door1) (1))
         (locked door1)
         (= (walls) (set-index walls true 2 2))
         (= (walls) (set-index walls true 2 4))
         (= (xloc gem1) (5))
         (= (yloc gem1) (2))
         (= (walls) (set-index walls true 3 2))
         (= (xloc key1) (3))
         (= (yloc key1) (3))
         (= (walls) (set-index walls true 3 4))
         (= (walls) (set-index walls true 4 2))
         (= (walls) (set-index walls true 4 4))
         (= (xloc gem2) (5))
         (= (yloc gem2) (4))
         (= (xloc door2) (4))
         (= (yloc door2) (5))
         (locked door2)
         (= (xpos) (1))
         (= (ypos) (3)))
  (:goal (has gem2))
)
