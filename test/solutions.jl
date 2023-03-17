@testset "Solutions" begin

using SymbolicPlanners:
    get_value, get_action_values, get_action_probs, get_action_prob

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

@test copy(sol) == sol

end

@testset "Path Search Solution" begin

plan = @pddl("(pick-up a)", "(stack a b)", "(pick-up c)", "(stack c a)")
trajectory = PDDL.simulate(blocksworld, bw_state, plan)
sol = PathSearchSolution(:success, plan, trajectory)

@test collect(sol) == plan
@test sol[1] == pddl"(pick-up a)"
@test get_action(sol, 1) == pddl"(pick-up a)"
@test length(sol) == 4
@test eltype(sol) == Term

no_op = convert(Term, PDDL.no_op)
@test get_action(sol, trajectory[3]) == plan[3]
@test get_action(sol, trajectory[end]) == no_op
@test get_action_probs(sol, trajectory[2]) == Dict(plan[2] => 1.0)
@test get_action_probs(sol, trajectory[end]) == Dict(no_op => 1.0)
@test get_action_prob(sol, trajectory[1], plan[1]) == 1.0

@test copy(sol) == sol

end

@testset "Random Policy" begin

sol = RandomPolicy(blocksworld)
@test get_action(sol, bw_state) in bw_init_actions
@test rand_action(sol, bw_state) in bw_init_actions

n_actions = length(bw_init_actions)
probs = Dict(a => 1.0 / n_actions for a in bw_init_actions)
@test get_action_probs(sol, bw_state) == probs
@test get_action_prob(sol, bw_state, pddl"(pick-up a)") == 1.0 / n_actions

@test copy(sol) == sol

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
@test get_action_values(sol, bw_state) == bw_init_q

probs = Dict(a => a == pddl"(pick-up a)" ? 1.0 : 0.0 for a in bw_init_actions)
@test get_action_probs(sol, bw_state) == probs
@test get_action_prob(sol, bw_state, pddl"(pick-up a)") == 1.0 
@test get_action_prob(sol, bw_state, pddl"(pick-up b)") == 0.0 

@test copy(sol) == sol

end

@testset "Tabular Value Policy" begin

sol = TabularVPolicy(blocksworld, bw_spec)
sol.V[hash(bw_state)] = bw_init_v
for (act, val) in bw_init_q
    next_state = transition(blocksworld, bw_state, act)
    sol.V[hash(next_state)] = val + 1
end

@test get_action(sol, bw_state) == pddl"(pick-up a)"
@test rand_action(sol, bw_state) == pddl"(pick-up a)"
@test best_action(sol, bw_state) == pddl"(pick-up a)"

@test get_value(sol, bw_state) == -4.0
@test get_value(sol, bw_state, pddl"(pick-up b)") == -6.0
@test get_action_values(sol, bw_state) == bw_init_q

probs = Dict(a => a == pddl"(pick-up a)" ? 1.0 : 0.0 for a in bw_init_actions)
@test get_action_probs(sol, bw_state) == probs
@test get_action_prob(sol, bw_state, pddl"(pick-up a)") == 1.0 
@test get_action_prob(sol, bw_state, pddl"(pick-up b)") == 0.0 

@test copy(sol) == sol

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

probs = Dict(a => a == pddl"(pick-up a)" ? 1.0 : 0.0 for a in bw_init_actions)
@test get_action_probs(sol, bw_state) == probs
@test get_action_prob(sol, bw_state, pddl"(pick-up a)") == 1.0 
@test get_action_prob(sol, bw_state, pddl"(pick-up b)") == 0.0 

@test copy(sol) == sol

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

probs = SymbolicPlanners.softmax(collect(values(bw_init_q)))
probs = Dict(zip(keys(bw_init_q), probs))
@test get_action_probs(sol, bw_state) == probs
act_prob = probs[pddl"(pick-up a)"]
@test get_action_prob(sol, bw_state, pddl"(pick-up a)") ≈ act_prob

@test copy(sol) == sol

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

probs = Dict(a => 0.1 / length(bw_init_actions) for a in bw_init_actions)
probs[pddl"(pick-up a)"] += 0.9
@test get_action_probs(sol, bw_state) == probs
act_prob = probs[pddl"(pick-up a)"]
@test get_action_prob(sol, bw_state, pddl"(pick-up a)") ≈ act_prob

@test copy(sol) == sol

end

end
