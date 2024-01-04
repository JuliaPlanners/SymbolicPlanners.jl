;;; bw-large-a
;;;
;;; Initial:  3/2/1   5/4   9/8/7/6
;;; Goal:     1/5     8/9/4/    2/3/7/6
;;; Length: 12

(define (problem bw-large-a)
  (:domain prodigy-bw)
  (:length (:parallel 12) (:serial 12))
  (:objects 1 2 3 4 5 6 7 8 9)
  (:init (arm-empty)
	 (on 3 2)
	 (on 2 1)
	 (on-table 1)
	 (on 5 4)
	 (on-table 4)
	 (on 9 8)
	 (on 8 7)
	 (on 7 6)
	 (on-table 6)
	 (clear 3)
	 (clear 5)
	 (clear 9))
  (:goal (and
	  (on 1 5)
	  (on-table 5)
	  (on 8 9)
	  (on 9 4)
	  (on-table 4)
	  (on 2 3)
	  (on 3 7)
	  (on 7 6)
	  (on-table 6)
	  (clear 1)
	  (clear 8)
	  (clear 2)
	  )))
