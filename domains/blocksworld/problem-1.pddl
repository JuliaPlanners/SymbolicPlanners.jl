(define (problem block_words)
	(:domain blocksworld)
	(:objects
		d r a w o e p c - block
	)
	(:init
		(handempty)
		(clear o)
		(ontable o)
		(clear r)
		(on r p)
		(ontable p)
		(clear e)
		(ontable e)
		(clear d)
		(on d a)
		(on a c)
		(ontable c)
		(clear w)
		(ontable w)
	)
	(:goal (and
		;; power
		(clear p) (ontable r) (on p o) (on o w) (on w e) (on e r)
	))
)
