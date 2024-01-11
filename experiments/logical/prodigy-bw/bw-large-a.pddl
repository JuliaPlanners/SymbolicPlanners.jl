;;; bw-large-a
;;;
;;; Initial:  b3/b2/b1   b5/b4   b9/b8/b7/b6
;;; Goal:     b1/b5     b8/b9/b4/    b2/b3/b7/b6
;;; Length:   12

(define (problem bw-large-a)
  (:domain blocks)
  (:length (:parallel 12) (:serial 12))
  (:objects b1 b2 b3 b4 b5 b6 b7 b8 b9 - block)
  (:init (arm-empty)
	 (on b3 b2)
	 (on b2 b1)
	 (on-table b1)
	 (on b5 b4)
	 (on-table b4)
	 (on b9 b8)
	 (on b8 b7)
	 (on b7 b6)
	 (on-table b6)
	 (clear b3)
	 (clear b5)
	 (clear b9))
  (:goal (and
	  (on b1 b5)
	  (on-table b5)
	  (on b8 b9)
	  (on b9 b4)
	  (on-table b4)
	  (on b2 b3)
	  (on b3 b7)
	  (on b7 b6)
	  (on-table b6)
	  (clear b1)
	  (clear b8)
	  (clear b2)
	  )))
