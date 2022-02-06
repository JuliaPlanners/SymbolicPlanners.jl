;; ASCII ;;
; W: wall, D: door, k: key, g: gem, G: goal-gem, s: start, .: empty
; ...D.
; .W.Wg
; sWkW.
; .W.WG
; ...D.
(define (problem doors-keys-gems-2)
  (:domain doors-keys-gems)
  (:objects key1 - key door1 door2 - door gem1 gem2 - gem)
  (:init (locked door1)
         (locked door2)
         (= (walls)
            (transpose (bit-mat
               (bit-vec 0 0 0 0 0)
               (bit-vec 0 1 0 1 0)
               (bit-vec 0 1 0 1 0)
               (bit-vec 0 1 0 1 0)
               (bit-vec 0 0 0 0 0))))
         (= (xloc key1) 3)
         (= (xloc door1) 4)
         (= (xloc gem1) 5)
         (= (xloc gem2) 5)
         (= (xloc door2) 4)
         (= (yloc key1) 3)
         (= (yloc door1) 1)
         (= (yloc gem1) 2)
         (= (yloc gem2) 4)
         (= (yloc door2) 5)
         (= (xpos) 1)
         (= (ypos) 3))
  (:goal (has gem2))
)
