using Test
using PDDL
import SymbolicPlanners as SP

@testset "Cabbage - PDDL issue #15" begin
    planner = SP.BreadthFirstPlanner()

    domain = load_domain("cabbage_domain.pddl")
    problem = load_problem("cabbage_problem.pddl")

    state = initstate(domain, problem)

    goal = SP.StateConstrainedGoal(problem)

    plan = planner(domain, state, goal)

    sol1 = [
        pddl"(ship-goat left right)",
        pddl"(ship-self right left)",
        pddl"(ship-cabbage left right)",
        pddl"(ship-goat right left)",
        pddl"(ship-wolf left right)",
        pddl"(ship-self right left)",
        pddl"(ship-goat left right)",
    ]

    sol2 = [
        pddl"(ship-goat left right)",
        pddl"(ship-self right left)",
        pddl"(ship-wolf left right)",
        pddl"(ship-goat right left)",
        pddl"(ship-cabbage left right)",
        pddl"(ship-self right left)",
        pddl"(ship-goat left right)",
    ]
    @test collect(plan) == sol1 || collect(plan) == sol2

end