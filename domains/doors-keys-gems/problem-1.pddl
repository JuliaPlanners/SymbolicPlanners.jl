;; ASCII ;;
; W: wall, D: door, k: key, g: gem, G: goal-gem, s: start, .: empty
; sWG
; kW.
; .D.
(define (problem doors-keys-gems-1)
  (:domain doors-keys-gems)
  (:objects key1 - key door1 - door gem1 - gem)
  (:init (locked door1)
         (= (walls)
            (transpose (bit-mat
               (bit-vec 0 1 0)
               (bit-vec 0 1 0)
               (bit-vec 0 0 0))))
         (= (xloc key1) 1)
         (= (xloc gem1) 3)
         (= (xloc door1) 2)
         (= (yloc key1) 2)
         (= (yloc gem1) 1)
         (= (yloc door1) 3)
         (= (xpos) 1)
         (= (ypos) 1))
  (:goal (has gem1))
)
