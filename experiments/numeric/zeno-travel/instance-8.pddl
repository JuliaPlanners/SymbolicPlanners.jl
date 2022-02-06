(define (problem ZTRAVEL-3-6-5)
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
	city0 - city
	city1 - city
	city2 - city
	city3 - city
	city4 - city
	)
(:init
	(at plane1 city0)
	(= (capacity plane1) 11591)
	(= (fuel plane1) 1371)
	(= (slow-burn plane1) 4)
	(= (fast-burn plane1) 9)
	(= (onboard plane1) 0)
	(= (zoom-limit plane1) 2)
	(at plane2 city3)
	(= (capacity plane2) 2717)
	(= (fuel plane2) 644)
	(= (slow-burn plane2) 1)
	(= (fast-burn plane2) 3)
	(= (onboard plane2) 0)
	(= (zoom-limit plane2) 7)
	(at plane3 city0)
	(= (capacity plane3) 5164)
	(= (fuel plane3) 1578)
	(= (slow-burn plane3) 2)
	(= (fast-burn plane3) 6)
	(= (onboard plane3) 0)
	(= (zoom-limit plane3) 7)
	(at person1 city1)
	(at person2 city0)
	(at person3 city2)
	(at person4 city0)
	(at person5 city3)
	(at person6 city4)
	(= (distance city0 city0) 0)
	(= (distance city0 city1) 900)
	(= (distance city0 city2) 666)
	(= (distance city0 city3) 731)
	(= (distance city0 city4) 878)
	(= (distance city1 city0) 900)
	(= (distance city1 city1) 0)
	(= (distance city1 city2) 943)
	(= (distance city1 city3) 823)
	(= (distance city1 city4) 580)
	(= (distance city2 city0) 666)
	(= (distance city2 city1) 943)
	(= (distance city2 city2) 0)
	(= (distance city2 city3) 757)
	(= (distance city2 city4) 891)
	(= (distance city3 city0) 731)
	(= (distance city3 city1) 823)
	(= (distance city3 city2) 757)
	(= (distance city3 city3) 0)
	(= (distance city3 city4) 556)
	(= (distance city4 city0) 878)
	(= (distance city4 city1) 580)
	(= (distance city4 city2) 891)
	(= (distance city4 city3) 556)
	(= (distance city4 city4) 0)
	(= (total-fuel-used) 0)
)
(:goal (and
	(at plane1 city3)
	(at person1 city0)
	(at person2 city0)
	(at person3 city1)
	(at person4 city0)
	(at person5 city3)
	(at person6 city2)
	))

(:metric minimize (+ (* 5 (total-time))  (* 3 (total-fuel-used))))
)
