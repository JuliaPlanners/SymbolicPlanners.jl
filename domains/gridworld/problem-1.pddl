;; ASCII ;;
; W: wall, g: goal, s: start, .: empty
; sWg
; .W.
; .D.
(define (problem gridworld-1)
  (:domain gridworld)
  (:init (= (walls)
            (transpose (bit-mat
               (bit-vec 0 1 0)
               (bit-vec 0 1 0)
               (bit-vec 0 0 0))))
         (= (xpos) 1)
         (= (ypos) 1))
  (:goal (and (= (xpos) 3) (= (ypos) 1)))
)
