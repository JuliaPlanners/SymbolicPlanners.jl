;; ASCII ;;
; W: wall, g: goal, s: start, .: empty
; .....
; .W.W.
; sW.Wg
; .W.W.
; .....
(define (problem gridworld-2)
  (:domain gridworld)
  (:init (= (walls)
            (transpose (bit-mat
               (bit-vec 0 0 0 0 0)
               (bit-vec 0 1 0 1 0)
               (bit-vec 0 1 0 1 0)
               (bit-vec 0 1 0 1 0)
               (bit-vec 0 0 0 0 0))))
         (= (xpos) 1)
         (= (ypos) 3))
  (:goal (and (= (xpos) 5) (= (ypos) 3)))
)
