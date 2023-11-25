(define (problem ZTRAVEL-3-7-5)
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
	)
(:init
	(at plane1 city2)
	(= (capacity plane1) 2483)
	(= (fuel plane1) 180)
	(= (slow-burn plane1) 1)
	(= (fast-burn plane1) 2)
	(= (onboard plane1) 0)
	(= (zoom-limit plane1) 9)
	(at plane2 city2)
	(= (capacity plane2) 14961)
	(= (fuel plane2) 3313)
	(= (slow-burn plane2) 5)
	(= (fast-burn plane2) 15)
	(= (onboard plane2) 0)
	(= (zoom-limit plane2) 1)
	(at plane3 city1)
	(= (capacity plane3) 6665)
	(= (fuel plane3) 259)
	(= (slow-burn plane3) 3)
	(= (fast-burn plane3) 9)
	(= (onboard plane3) 0)
	(= (zoom-limit plane3) 3)
	(at person1 city4)
	(at person2 city1)
	(at person3 city2)
	(at person4 city0)
	(at person5 city4)
	(at person6 city3)
	(at person7 city3)
	(= (distance city0 city0) 0)
	(= (distance city0 city1) 826)
	(= (distance city0 city2) 989)
	(= (distance city0 city3) 633)
	(= (distance city0 city4) 611)
	(= (distance city1 city0) 826)
	(= (distance city1 city1) 0)
	(= (distance city1 city2) 963)
	(= (distance city1 city3) 889)
	(= (distance city1 city4) 856)
	(= (distance city2 city0) 989)
	(= (distance city2 city1) 963)
	(= (distance city2 city2) 0)
	(= (distance city2 city3) 789)
	(= (distance city2 city4) 768)
	(= (distance city3 city0) 633)
	(= (distance city3 city1) 889)
	(= (distance city3 city2) 789)
	(= (distance city3 city3) 0)
	(= (distance city3 city4) 865)
	(= (distance city4 city0) 611)
	(= (distance city4 city1) 856)
	(= (distance city4 city2) 768)
	(= (distance city4 city3) 865)
	(= (distance city4 city4) 0)
	(= (total-fuel-used) 0)
)
(:goal (and
	(at person1 city2)
	(at person2 city0)
	(at person3 city4)
	(at person4 city3)
	(at person5 city1)
	(at person6 city4)
	(at person7 city4)
	))

(:metric minimize (+ (* 2 (total-time))  (* 1 (total-fuel-used))))
)
