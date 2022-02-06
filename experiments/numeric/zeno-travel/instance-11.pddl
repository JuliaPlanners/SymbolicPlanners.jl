(define (problem ZTRAVEL-3-7-6)
(:domain zeno-travel)
(:objects
	plane1 - aircraft
	plane2 - aircraft
	plane3 - aircraft
	person1 - person
	person2 - person
	person3 - person
	person4 - person
	person5 - person
	person6 - person
	person7 - person
	city0 - city
	city1 - city
	city2 - city
	city3 - city
	city4 - city
	city5 - city
	)
(:init
	(at plane1 city4)
	(= (capacity plane1) 5423)
	(= (fuel plane1) 1075)
	(= (slow-burn plane1) 2)
	(= (fast-burn plane1) 5)
	(= (onboard plane1) 0)
	(= (zoom-limit plane1) 3)
	(at plane2 city4)
	(= (capacity plane2) 3038)
	(= (fuel plane2) 45)
	(= (slow-burn plane2) 1)
	(= (fast-burn plane2) 2)
	(= (onboard plane2) 0)
	(= (zoom-limit plane2) 5)
	(at plane3 city1)
	(= (capacity plane3) 9837)
	(= (fuel plane3) 3036)
	(= (slow-burn plane3) 4)
	(= (fast-burn plane3) 10)
	(= (onboard plane3) 0)
	(= (zoom-limit plane3) 4)
	(at person1 city4)
	(at person2 city2)
	(at person3 city2)
	(at person4 city0)
	(at person5 city2)
	(at person6 city2)
	(at person7 city5)
	(= (distance city0 city0) 0)
	(= (distance city0 city1) 941)
	(= (distance city0 city2) 897)
	(= (distance city0 city3) 628)
	(= (distance city0 city4) 808)
	(= (distance city0 city5) 713)
	(= (distance city1 city0) 941)
	(= (distance city1 city1) 0)
	(= (distance city1 city2) 999)
	(= (distance city1 city3) 870)
	(= (distance city1 city4) 574)
	(= (distance city1 city5) 728)
	(= (distance city2 city0) 897)
	(= (distance city2 city1) 999)
	(= (distance city2 city2) 0)
	(= (distance city2 city3) 718)
	(= (distance city2 city4) 560)
	(= (distance city2 city5) 800)
	(= (distance city3 city0) 628)
	(= (distance city3 city1) 870)
	(= (distance city3 city2) 718)
	(= (distance city3 city3) 0)
	(= (distance city3 city4) 920)
	(= (distance city3 city5) 778)
	(= (distance city4 city0) 808)
	(= (distance city4 city1) 574)
	(= (distance city4 city2) 560)
	(= (distance city4 city3) 920)
	(= (distance city4 city4) 0)
	(= (distance city4 city5) 556)
	(= (distance city5 city0) 713)
	(= (distance city5 city1) 728)
	(= (distance city5 city2) 800)
	(= (distance city5 city3) 778)
	(= (distance city5 city4) 556)
	(= (distance city5 city5) 0)
	(= (total-fuel-used) 0)
)
(:goal (and
	(at plane1 city1)
	(at person1 city4)
	(at person2 city1)
	(at person3 city2)
	(at person4 city2)
	(at person5 city2)
	(at person6 city4)
	(at person7 city0)
	))

(:metric minimize (+ (* 4 (total-time))  (* 3 (total-fuel-used))))
)
