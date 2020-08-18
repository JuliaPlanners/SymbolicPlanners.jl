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
  (:init
    (= width 8) (= height 8)
    (= xpos 1) (= ypos 1)
    (wall 1 2) (wall 2 2) (wall 3 2)
    (wall 5 1) (wall 5 2) (wall 5 3) (wall 5 4)
    (wall 7 2) (wall 7 3) (wall 7 4) (wall 7 5) (wall 7 6)
    (wall 2 4) (wall 2 5) (wall 2 6) (wall 2 7)
    (wall 2 6) (wall 3 6) (wall 4 6) (wall 5 6) (wall 6 6) (wall 7 6)
    (wall 4 8) (wall 6 7)
  )
  (:goal (and (= xpos 7) (= ypos 8)))
)
