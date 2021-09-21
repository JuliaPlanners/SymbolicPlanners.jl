;; ASCII ;;
; W: wall, g: goal, s: start, .: empty
; .....
; .W.W.
; sW.Wg
; .W.W.
; .....
(define (problem gridworld-2)
  (:domain gridworld)
  (:init (= (wallgrid) (new-bit-matrix false 5 5))
         (= (wallgrid) (set-index wallgrid true 2 2))
         (= (wallgrid) (set-index wallgrid true 2 4))
         (= (wallgrid) (set-index wallgrid true 3 2))
         (= (wallgrid) (set-index wallgrid true 3 4))
         (= (wallgrid) (set-index wallgrid true 4 2))
         (= (wallgrid) (set-index wallgrid true 4 4))
         (= (xpos) (1))
         (= (ypos) (3)))
  (:goal (and (= (xpos) (5)) (= (ypos) (3))))
)
