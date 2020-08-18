(define (problem block_words)
	(:domain blocksworld)
	(:objects
		s t a r h c u k - block
	)
	(:init
		(handempty)
		(clear t)
		(ontable t)
		(clear u)
		(ontable u)
		(clear r)
		(ontable r)
		(clear k)
		(ontable k)
		(clear c)
		(ontable c)
		(clear s)
		(on s a)
		(on a h)
		(ontable h)
	)
	(:goal (and
		;; starch
		(clear s) (ontable h) (on s t) (on t a) (on a r) (on r c) (on c h)
	))
)
