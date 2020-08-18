;; ASCII ;;
; W: wall, g: goal, s: start, .: empty
; sWg
; .W.
; .D.
(define (problem gridworld-1)
  (:domain gridworld)
  (:init
    (= width 3) (= height 3)
    (= xpos 1) (= ypos 3)
    (wall 2 3) (wall 2 2)
  )
  (:goal (and (= xpos 3) (= ypos 3)))
)
