(define (problem ZTRAVEL-2-4-3)
(:domain zeno-travel)
(:objects
	plane1 - aircraft
	plane2 - aircraft
	person1 - person
	person2 - person
	person3 - person
	person4 - person
	city0 - city
	city1 - city
	city2 - city
	)
(:init
	(at plane1 city0)
	(= (capacity plane1) 8873)
	(= (fuel plane1) 2328)
	(= (slow-burn plane1) 3)
	(= (fast-burn plane1) 7)
	(= (onboard plane1) 0)
	(= (zoom-limit plane1) 8)
	(at plane2 city2)
	(= (capacity plane2) 9074)
	(= (fuel plane2) 3624)
	(= (slow-burn plane2) 4)
	(= (fast-burn plane2) 10)
	(= (onboard plane2) 0)
	(= (zoom-limit plane2) 2)
	(at person1 city0)
	(at person2 city0)
	(at person3 city1)
	(at person4 city1)
	(= (distance city0 city0) 0)
	(= (distance city0 city1) 750)
	(= (distance city0 city2) 532)
	(= (distance city1 city0) 750)
	(= (distance city1 city1) 0)
	(= (distance city1 city2) 768)
	(= (distance city2 city0) 532)
	(= (distance city2 city1) 768)
	(= (distance city2 city2) 0)
	(= (total-fuel-used) 0)
)
(:goal (and
	(at plane2 city2)
	(at person1 city1)
	(at person2 city0)
	(at person3 city0)
	(at person4 city1)
	))

(:metric minimize (+ (* 1 (total-time))  (* 1 (total-fuel-used))))
)
