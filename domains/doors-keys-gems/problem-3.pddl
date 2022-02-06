;; ASCII ;;
; W: wall, D: door, k: key, g: gem, G: goal-gem, s: start, .: empty
; g..W...g
; k..D.Wk.
; .WWWWWWD
; .W..D.W.
; .W..W.W.
; ....W.W.
; WWW.W.W.
; s...W..G
(define (problem doors-keys-gems-3)
  (:domain doors-keys-gems)
  (:objects key1 key2 - key
            door1 door2 door3 - door
            gem1 gem2 gem3 - gem)
  (:init (locked door3)
         (locked door1)
         (locked door2)
         (= (walls)
            (transpose (bit-mat
               (bit-vec 0 0 0 1 0 0 0 0)
               (bit-vec 0 0 0 0 0 1 0 0)
               (bit-vec 0 1 1 1 1 1 1 0)
               (bit-vec 0 1 0 0 0 0 1 0)
               (bit-vec 0 1 0 0 1 0 1 0)
               (bit-vec 0 0 0 0 1 0 1 0)
               (bit-vec 1 1 1 0 1 0 1 0)
               (bit-vec 0 0 0 0 1 0 0 0))))
         (= (xloc key1) 1)
         (= (xloc key2) 7)
         (= (xloc door3) 5)
         (= (xloc gem1) 1)
         (= (xloc gem2) 8)
         (= (xloc door1) 4)
         (= (xloc door2) 8)
         (= (xloc gem3) 8)
         (= (yloc key1) 2)
         (= (yloc key2) 2)
         (= (yloc door3) 4)
         (= (yloc gem1) 1)
         (= (yloc gem2) 1)
         (= (yloc door1) 2)
         (= (yloc door2) 3)
         (= (yloc gem3) 8)
         (= (xpos) 1)
         (= (ypos) 8))
  (:goal (has gem3))
)
