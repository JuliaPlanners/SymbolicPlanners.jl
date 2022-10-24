@testset "Solutions" begin

using SymbolicPlanners: get_value, get_action_values

# Available actions in initial Blocksworld state
bw_init_actions = collect(available(blocksworld, bw_state))

# Value and Q-values for initial Blocksworld state
bw_init_v = -4.0
bw_init_q = Dict{Term,Float64}(
    pddl"(pick-up a)" => -4.0,
    pddl"(pick-up b)" => -6.0,
    pddl"(pick-up c)" => -6.0
)

@testset "Ordered Plan" begin

sol = OrderedPlan(@pddl("(pick-up a)", "(stack a b)",
                        "(pick-up c)", "(stack c a)"))
@test collect(sol) == @pddl("(pick-up a)", "(stack a b)",
                            "(pick-up c)", "(stack c a)")
@test sol[1] == pddl"(pick-up a)"
@test get_action(sol, 1) == pddl"(pick-up a)"
@test length(sol) == 4
@test eltype(sol) == Term

end

@testset "Random Policy" begin

sol = RandomPolicy(blocksworld)
@test get_action(sol, bw_state) in bw_init_actions
@test rand_action(sol, bw_state) in bw_init_actions

end

@testset "Tabular Policy" begin

sol = TabularPolicy()
sol.V[hash(bw_state)] = bw_init_v
sol.Q[hash(bw_state)] = bw_init_q

@test get_action(sol, bw_state) == pddl"(pick-up a)"
@test rand_action(sol, bw_state) == pddl"(pick-up a)"
@test best_action(sol, bw_state) == pddl"(pick-up a)"

@test get_value(sol, bw_state) == -4.0
@test get_value(sol, bw_state, pddl"(pick-up b)") == -6.0
@test Dict(get_action_values(sol, bw_state)) == bw_init_q

end

@testset "Functional Value Policy" begin

planner = AStarPlanner(HMax())
heuristic = PlannerHeuristic(planner)
sol = FunctionalVPolicy(heuristic, blocksworld, bw_spec)

@test get_action(sol, bw_state) == pddl"(pick-up a)"
@test rand_action(sol, bw_state) == pddl"(pick-up a)"
@test best_action(sol, bw_state) == pddl"(pick-up a)"

@test get_value(sol, bw_state) == -4.0
@test get_value(sol, bw_state, pddl"(pick-up b)") == -6.0
@test Dict(get_action_values(sol, bw_state)) == bw_init_q

end

@testset "Boltzmann Policy" begin

planner = AStarPlanner(HMax())
heuristic = PlannerHeuristic(planner)
sol = FunctionalVPolicy(heuristic, blocksworld, bw_spec)
sol = BoltzmannPolicy(sol, 1.0)

@test get_action(sol, bw_state) in bw_init_actions
@test rand_action(sol, bw_state) in bw_init_actions
@test best_action(sol, bw_state) == pddl"(pick-up a)"

@test get_value(sol, bw_state) == -4.0
@test get_value(sol, bw_state, pddl"(pick-up b)") == -6.0
@test Dict(get_action_values(sol, bw_state)) == bw_init_q

end

@testset "Epsilon-Greedy Policy" begin

planner = AStarPlanner(HMax())
heuristic = PlannerHeuristic(planner)
sol = FunctionalVPolicy(heuristic, blocksworld, bw_spec)
sol = EpsilonGreedyPolicy(blocksworld, sol, 0.1)

@test get_action(sol, bw_state) in bw_init_actions
@test rand_action(sol, bw_state) in bw_init_actions
@test best_action(sol, bw_state) == pddl"(pick-up a)"

@test get_value(sol, bw_state) == -4.0
@test get_value(sol, bw_state, pddl"(pick-up b)") == -6.0
@test Dict(get_action_values(sol, bw_state)) == bw_init_q

end

end
