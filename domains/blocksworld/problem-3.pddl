(define (problem block_words)
	(:domain blocksworld)
	(:objects
		m o t h e r w a - block
	)
	(:init
		(handempty)
		(clear m)
		(on m o)
		(ontable o)
		(clear e)
		(ontable e)
		(clear t)
		(on t w)
		(ontable w)
		(clear h)
		(ontable h)
		(clear r)
		(on r a)
		(ontable a)
	)
	(:goal (and
		;; mother
		(clear m) (ontable r) (on m o) (on o t) (on t h) (on h e) (on e r)
	))
)
