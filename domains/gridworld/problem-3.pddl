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
(define (problem gridworld-problem)
  (:domain gridworld)
  (:init (= (wallgrid) (new-bit-matrix false 8 8))
         (= (wallgrid) (set-index wallgrid true 1 4))
         (= (wallgrid) (set-index wallgrid true 2 2))
         (= (wallgrid) (set-index wallgrid true 2 6))
         (= (wallgrid) (set-index wallgrid true 3 2))
         (= (wallgrid) (set-index wallgrid true 3 3))
         (= (wallgrid) (set-index wallgrid true 3 4))
         (= (wallgrid) (set-index wallgrid true 3 5))
         (= (wallgrid) (set-index wallgrid true 3 6))
         (= (wallgrid) (set-index wallgrid true 3 7))
         (= (wallgrid) (set-index wallgrid true 4 2))
         (= (wallgrid) (set-index wallgrid true 4 7))
         (= (wallgrid) (set-index wallgrid true 5 2))
         (= (wallgrid) (set-index wallgrid true 5 5))
         (= (wallgrid) (set-index wallgrid true 5 7))
         (= (wallgrid) (set-index wallgrid true 6 5))
         (= (wallgrid) (set-index wallgrid true 6 7))
         (= (wallgrid) (set-index wallgrid true 7 1))
         (= (wallgrid) (set-index wallgrid true 7 2))
         (= (wallgrid) (set-index wallgrid true 7 3))
         (= (wallgrid) (set-index wallgrid true 7 5))
         (= (wallgrid) (set-index wallgrid true 7 7))
         (= (wallgrid) (set-index wallgrid true 8 5))
         (= (xpos) (1))
         (= (ypos) (8)))
  (:goal (and (= (xpos) (7)) (= (ypos) (1))))
)
