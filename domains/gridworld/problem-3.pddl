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
  (:init (= (walls) (new-bit-matrix false 8 8))
         (= (walls) (set-index walls true 1 4))
         (= (walls) (set-index walls true 2 2))
         (= (walls) (set-index walls true 2 6))
         (= (walls) (set-index walls true 3 2))
         (= (walls) (set-index walls true 3 3))
         (= (walls) (set-index walls true 3 4))
         (= (walls) (set-index walls true 3 5))
         (= (walls) (set-index walls true 3 6))
         (= (walls) (set-index walls true 3 7))
         (= (walls) (set-index walls true 4 2))
         (= (walls) (set-index walls true 4 7))
         (= (walls) (set-index walls true 5 2))
         (= (walls) (set-index walls true 5 5))
         (= (walls) (set-index walls true 5 7))
         (= (walls) (set-index walls true 6 5))
         (= (walls) (set-index walls true 6 7))
         (= (walls) (set-index walls true 7 1))
         (= (walls) (set-index walls true 7 2))
         (= (walls) (set-index walls true 7 3))
         (= (walls) (set-index walls true 7 5))
         (= (walls) (set-index walls true 7 7))
         (= (walls) (set-index walls true 8 5))
         (= (xpos) (1))
         (= (ypos) (8)))
  (:goal (and (= (xpos) (7)) (= (ypos) (1))))
)
