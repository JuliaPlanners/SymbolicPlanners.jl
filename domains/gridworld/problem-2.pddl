;; ASCII ;;
; W: wall, g: goal, s: start, .: empty
; .....
; .W.W.
; sW.Wg
; .W.W.
; .....
(define (problem gridworld-2)
  (:domain gridworld)
  (:init
    (= width 5) (= height 5)
    (= xpos 1) (= ypos 3)
    (wall 2 2) (wall 2 3) (wall 2 4)
    (wall 4 2) (wall 4 3) (wall 4 4)
  )
  (:goal (and (= xpos 5) (= ypos 3)))
)
