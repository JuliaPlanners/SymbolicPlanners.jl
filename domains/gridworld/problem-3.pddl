;; ASCII ;;
; W: wall, g: goal, s: start, .: empty
; ...W..g.
; .W...W..
; .WWWWWW.
; .W....W.
; .W..W.W.
; ....W.W.
; WWW.W.W.
; s...W...
(define (problem gridworld-3)
  (:domain gridworld)
  (:init (= (walls)
            (transpose (bit-mat ; transpose from column-order to row-order
               (bit-vec 0 0 0 1 0 0 0 0)
               (bit-vec 0 1 0 0 0 1 0 0)
               (bit-vec 0 1 1 1 1 1 1 0)
               (bit-vec 0 1 0 0 0 0 1 0)
               (bit-vec 0 1 0 0 1 0 1 0)
               (bit-vec 0 0 0 0 1 0 1 0)
               (bit-vec 1 1 1 0 1 0 1 0)
               (bit-vec 0 0 0 0 1 0 0 0))))
         (= (xpos) 1)
         (= (ypos) 8))
  (:goal (and (= (xpos) 7) (= (ypos) 1)))
)
