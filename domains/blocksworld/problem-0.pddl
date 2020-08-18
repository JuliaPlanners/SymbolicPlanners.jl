(define (problem block_words)
	(:domain blocksworld)
	(:objects
		a b c - block
	)
	(:init
		(handempty)
		(ontable a)
		(ontable b)
		(ontable c)
		(clear a)
		(clear b)
		(clear c)
	)
	(:goal (and
		;; cab
		(clear c) (ontable b) (on c a) (on a b)
	))
)
