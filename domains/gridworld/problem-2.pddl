;; ASCII ;;
; W: wall, g: goal, s: start, .: empty
; .....
; .W.W.
; sW.Wg
; .W.W.
; .....
(define (problem gridworld-2)
  (:domain gridworld)
  (:init (= (walls) (new-bit-matrix false 5 5))
         (= (walls) (set-index walls true 2 2))
         (= (walls) (set-index walls true 2 4))
         (= (walls) (set-index walls true 3 2))
         (= (walls) (set-index walls true 3 4))
         (= (walls) (set-index walls true 4 2))
         (= (walls) (set-index walls true 4 4))
         (= (xpos) (1))
         (= (ypos) (3)))
  (:goal (and (= (xpos) (5)) (= (ypos) (3))))
)
