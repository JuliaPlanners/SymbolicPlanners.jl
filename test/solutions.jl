@testset "Solutions" begin

using SymbolicPlanners:
    get_value, get_action_values, get_action_probs, get_action_prob,
    has_values, has_cached_value, has_cached_action_values,
    get_mixture_weights, PathNode, LinkedNodeRef

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

@test has_values(sol) == false

@test copy(sol) == sol

end

@testset "External Plan" begin

sol = ExternalPlan(@pddl("(pick-up a)", "(stack a b)",
                         "(pick-up c)", "(stack c a)"))
@test collect(sol) == @pddl("(pick-up a)", "(stack a b)",
                            "(pick-up c)", "(stack c a)")
@test sol[1] == pddl"(pick-up a)"
@test get_action(sol, 1) == pddl"(pick-up a)"
@test length(sol) == 4
@test eltype(sol) == Term

@test has_values(sol) == false

@test copy(sol) == sol

end

@testset "Path Search Solution" begin

plan = @pddl("(pick-up a)", "(stack a b)", "(pick-up c)", "(stack c a)")
trajectory = PDDL.simulate(blocksworld, bw_state, plan)

path_nodes = map(enumerate(trajectory)) do (i, state)
    id = hash(state)
    path_cost = Float32(i - 1)
    parent_id = i > 1 ? hash(trajectory[i - 1]) : nothing
    parent_action = i > 1 ? plan[i - 1] : nothing
    parent_ref = i > 1 ? LinkedNodeRef(parent_id, parent_action) : nothing
    return PathNode(id, state, path_cost, parent_ref)
end
search_tree = Dict(node.id => node for node in path_nodes)
queue = [trajectory[end]]

sol = PathSearchSolution(:success, collect(Term, plan), trajectory, -1,
                         search_tree, queue, UInt[])

node_id = hash(trajectory[end])
@test SymbolicPlanners.reconstruct(node_id, search_tree) == (plan, trajectory)

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
@test get_action_prob(sol, trajectory[1], pddl"(pick-up b)") == 0.0
@test get_action_prob(sol, trajectory[1], pddl"(pick-up z)") == 0.0

@test has_values(sol) == false

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
@test get_action_prob(sol, bw_state, pddl"(pick-up z)") == 0.0

@test has_values(sol) == false

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

next_bw_state = transition(blocksworld, bw_state, pddl"(pick-up a)")
@test has_cached_value(sol, bw_state)
@test !has_cached_value(sol, next_bw_state)
@test has_cached_value(sol, bw_state, pddl"(pick-up a)")
@test !has_cached_value(sol, bw_state, pddl"(pick-up z)")
@test !has_cached_value(sol, next_bw_state, pddl"(put-down a)")
@test has_cached_action_values(sol, bw_state)
@test !has_cached_action_values(sol, next_bw_state)

probs = Dict(a => a == pddl"(pick-up a)" ? 1.0 : 0.0 for a in bw_init_actions)
@test get_action_probs(sol, bw_state) == probs
@test get_action_prob(sol, bw_state, pddl"(pick-up a)") == 1.0
@test get_action_prob(sol, bw_state, pddl"(pick-up b)") == 0.0
@test get_action_prob(sol, bw_state, pddl"(pick-up z)") == 0.0

@test has_values(sol) == true

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

next_bw_state = transition(blocksworld, bw_state, pddl"(pick-up a)")
next_next_bw_state = transition(blocksworld, next_bw_state, pddl"(stack a b)")
@test has_cached_value(sol, bw_state)
@test has_cached_value(sol, next_bw_state)
@test !has_cached_value(sol, next_next_bw_state)
@test has_cached_value(sol, bw_state, pddl"(pick-up a)")
@test !has_cached_value(sol, bw_state, pddl"(pick-up z)")
@test !has_cached_value(sol, next_bw_state, pddl"(stack a b)")
@test has_cached_action_values(sol, bw_state)
@test !has_cached_action_values(sol, next_bw_state)

probs = Dict(a => a == pddl"(pick-up a)" ? 1.0 : 0.0 for a in bw_init_actions)
@test get_action_probs(sol, bw_state) == probs
@test get_action_prob(sol, bw_state, pddl"(pick-up a)") == 1.0
@test get_action_prob(sol, bw_state, pddl"(pick-up b)") == 0.0
@test get_action_prob(sol, bw_state, pddl"(pick-up z)") == 0.0

@test has_values(sol) == true

@test copy(sol) == sol

end

@testset "Functional Value Policy" begin

planner = AStarPlanner(HMax())
heuristic = PlannerHeuristic(planner)
h_function(state::State) = -compute(heuristic, blocksworld, state, bw_spec)
sol = FunctionalVPolicy(h_function, blocksworld, bw_spec)

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

@test has_values(sol) == true

@test copy(sol) == sol

end

@testset "Heuristic Value Policy" begin

planner = AStarPlanner(HMax())
heuristic = PlannerHeuristic(planner)
sol = HeuristicVPolicy(heuristic, blocksworld, bw_spec)

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

@test has_values(sol) == true

@test copy(sol) == sol

end

@testset "Reusable Tree Policy" begin

sol = TabularVPolicy(blocksworld, bw_spec)
sol.V[hash(bw_state)] = bw_init_v
for (act, val) in bw_init_q
    next_state = transition(blocksworld, bw_state, act)
    sol.V[hash(next_state)] = val + 1
end

plan = @pddl("(pick-up a)", "(stack a b)", "(pick-up c)", "(stack c a)")
trajectory = PDDL.simulate(blocksworld, bw_state, plan)

path_nodes = map(enumerate(trajectory)) do (i, state)
    id = hash(state)
    path_cost = Float32(i - 1)
    parent_id = i > 1 ? hash(trajectory[i - 1]) : nothing
    parent_action = i > 1 ? plan[i - 1] : nothing
    parent_ref = i > 1 ? LinkedNodeRef(parent_id, parent_action) : nothing
    return PathNode(id, state, path_cost, parent_ref)
end
search_tree = Dict(node.id => node for node in path_nodes)
search_sol = PathSearchSolution(:success, collect(Term, plan), trajectory)

sol = ReusableTreePolicy{GenericState}(sol, search_sol)
SymbolicPlanners.insert_path!(sol, search_tree, hash(trajectory[end]), 4.0f0)

@test get_action(sol, bw_state) == pddl"(pick-up a)"
@test rand_action(sol, bw_state) == pddl"(pick-up a)"
@test best_action(sol, bw_state) == pddl"(pick-up a)"

@test get_action(sol, trajectory[1]) == plan[1]
@test get_action(sol, trajectory[2]) == plan[2]
@test get_action(sol, trajectory[3]) == plan[3]
@test get_action(sol, trajectory[4]) == plan[4]

@test get_value(sol, bw_state) == -4.0
@test get_value(sol, bw_state, pddl"(pick-up b)") == -6.0
@test get_action_values(sol, bw_state) == bw_init_q

next_bw_state = transition(blocksworld, bw_state, pddl"(pick-up a)")
next_next_bw_state = transition(blocksworld, next_bw_state, pddl"(stack a b)")
@test has_cached_value(sol, bw_state)
@test has_cached_value(sol, next_bw_state)
@test !has_cached_value(sol, next_next_bw_state)
@test has_cached_value(sol, bw_state, pddl"(pick-up a)")
@test !has_cached_value(sol, bw_state, pddl"(pick-up z)")
@test !has_cached_value(sol, next_bw_state, pddl"(stack a b)")
@test has_cached_action_values(sol, bw_state)
@test !has_cached_action_values(sol, next_bw_state)

probs = Dict(a => a == pddl"(pick-up a)" ? 1.0 : 0.0 for a in bw_init_actions)
@test get_action_probs(sol, bw_state) == probs
@test get_action_prob(sol, bw_state, pddl"(pick-up a)") == 1.0
@test get_action_prob(sol, bw_state, pddl"(pick-up b)") == 0.0
@test get_action_prob(sol, bw_state, pddl"(pick-up z)") == 0.0

@test has_values(sol) == true

@test copy(sol) == sol

end

@testset "Boltzmann Policy" begin

planner = AStarPlanner(HMax())
heuristic = PlannerHeuristic(planner)
sol = HeuristicVPolicy(heuristic, blocksworld, bw_spec)
sol = BoltzmannPolicy(sol, 1.0)

@test get_action(sol, bw_state) in bw_init_actions
@test rand_action(sol, bw_state) in bw_init_actions
@test best_action(sol, bw_state) == pddl"(pick-up a)"

@test get_value(sol, bw_state) == -4.0
@test get_value(sol, bw_state, pddl"(pick-up b)") == -6.0
@test Dict(get_action_values(sol, bw_state)) == bw_init_q

probs = SymbolicPlanners.softmax(collect(values(bw_init_q)))
probs = Dict(zip(keys(bw_init_q), probs))
@test all(probs[a] ≈ p for (a, p) in get_action_probs(sol, bw_state))
act_prob = probs[pddl"(pick-up a)"]
@test get_action_prob(sol, bw_state, pddl"(pick-up a)") ≈ act_prob
@test get_action_prob(sol, bw_state, pddl"(pick-up z)") == 0.0

sol = TabularPolicy()
sol.V[hash(bw_state)] = bw_init_v
sol.Q[hash(bw_state)] = copy(bw_init_q)
sol.Q[hash(bw_state)][pddl"(pick-up a)"] = -Inf
sol = BoltzmannPolicy(sol, 0.0)
probs = Dict(a => a == pddl"(pick-up a)" ? 0.0 : 0.5 for a in bw_init_actions)
@test all(probs[a] ≈ p for (a, p) in get_action_probs(sol, bw_state))
@test get_action_prob(sol, bw_state, pddl"(pick-up a)") == 0.0
@test get_action_prob(sol, bw_state, pddl"(pick-up b)") == 0.5
@test get_action_prob(sol, bw_state, pddl"(pick-up c)") == 0.5

@test has_values(sol) == true

@test copy(sol) == sol

end

@testset "Boltzmann Mixture Policy" begin

planner = AStarPlanner(HMax())
heuristic = PlannerHeuristic(planner)
sol = HeuristicVPolicy(heuristic, blocksworld, bw_spec)

sol = BoltzmannMixturePolicy(sol, [1.0, 2.0])
@test get_mixture_weights(sol) == [0.5, 0.5]
sol = BoltzmannMixturePolicy(sol, [1.0, 2.0], [0.4, 0.6])
@test get_mixture_weights(sol) == [0.4, 0.6]

@test get_action(sol, bw_state) in bw_init_actions
@test rand_action(sol, bw_state) in bw_init_actions
@test best_action(sol, bw_state) == pddl"(pick-up a)"

@test get_value(sol, bw_state) == -4.0
@test get_value(sol, bw_state, pddl"(pick-up b)") == -6.0
@test Dict(get_action_values(sol, bw_state)) == bw_init_q

probs_1 = SymbolicPlanners.softmax(collect(values(bw_init_q)))
probs_2 = SymbolicPlanners.softmax(collect(values(bw_init_q)) ./ 2.0)
probs = probs_1 * 0.4 + probs_2 * 0.6
probs = Dict(zip(keys(bw_init_q), probs))
@test all(probs[a] ≈ p for (a, p) in get_action_probs(sol, bw_state))
act_prob = probs[pddl"(pick-up a)"]
@test get_action_prob(sol, bw_state, pddl"(pick-up a)") ≈ act_prob
@test get_action_prob(sol, bw_state, pddl"(pick-up z)") == 0.0

act = pddl"(pick-up a)"
probs_1 = Dict(zip(keys(bw_init_q), probs_1))
probs_2 = Dict(zip(keys(bw_init_q), probs_2))
new_weights = [0.4 * probs_1[act], 0.6 * probs_2[act]]
new_weights = new_weights ./ sum(new_weights)
@test get_mixture_weights(sol, bw_state, act) ≈ new_weights

sol = TabularPolicy()
sol.V[hash(bw_state)] = bw_init_v
sol.Q[hash(bw_state)] = copy(bw_init_q)
sol.Q[hash(bw_state)][pddl"(pick-up a)"] = -6.0 - log(2)
sol = BoltzmannMixturePolicy(sol, [0.0, 1.0])
probs = Dict(a => a == pddl"(pick-up a)" ? 0.1 : 0.45 for a in bw_init_actions)
@test all(probs[a] ≈ p for (a, p) in get_action_probs(sol, bw_state))
@test get_action_prob(sol, bw_state, pddl"(pick-up a)") ≈ 0.1
@test get_action_prob(sol, bw_state, pddl"(pick-up b)") ≈ 0.45
@test get_action_prob(sol, bw_state, pddl"(pick-up c)") ≈ 0.45
new_weights = [0.5, 0.4] .* get_mixture_weights(sol)
new_weights = new_weights ./ sum(new_weights)
@test get_mixture_weights(sol, bw_state, pddl"(pick-up b)") ≈ new_weights

@test has_values(sol) == true

@test copy(sol) == sol

end

@testset "Epsilon Greedy Policy" begin

planner = AStarPlanner(HMax())
heuristic = PlannerHeuristic(planner)
sol = HeuristicVPolicy(heuristic, blocksworld, bw_spec)
sol = EpsilonGreedyPolicy(blocksworld, sol, 0.1)

@test get_action(sol, bw_state) in bw_init_actions
@test rand_action(sol, bw_state) in bw_init_actions
@test best_action(sol, bw_state) == pddl"(pick-up a)"

@test get_value(sol, bw_state) == -4.0
@test get_value(sol, bw_state, pddl"(pick-up b)") == -6.0
@test Dict(get_action_values(sol, bw_state)) == bw_init_q

probs = Dict(a => 0.1 / length(bw_init_actions) for a in bw_init_actions)
probs[pddl"(pick-up a)"] += 0.9
@test all(probs[a] ≈ p for (a, p) in get_action_probs(sol, bw_state))
act_prob = probs[pddl"(pick-up a)"]
@test get_action_prob(sol, bw_state, pddl"(pick-up a)") ≈ act_prob
@test get_action_prob(sol, bw_state, pddl"(pick-up z)") == 0.0

sol = TabularPolicy()
sol.V[hash(bw_state)] = bw_init_v
sol.Q[hash(bw_state)] = copy(bw_init_q)
sol.Q[hash(bw_state)][pddl"(pick-up a)"] = -8.0
sol = EpsilonGreedyPolicy(blocksworld, sol, 0.1)
probs = Dict(a => a == pddl"(pick-up a)" ? 0.1 / 3 : 0.45 + 0.1 / 3
             for a in bw_init_actions)
@test all(probs[a] ≈ p for (a, p) in get_action_probs(sol, bw_state))
@test get_action_prob(sol, bw_state, pddl"(pick-up a)") ≈ 0.1 / 3
@test get_action_prob(sol, bw_state, pddl"(pick-up b)") ≈ 0.45 + 0.1 / 3
@test get_action_prob(sol, bw_state, pddl"(pick-up c)") ≈ 0.45 + 0.1 / 3

@test has_values(sol) == true

plan = @pddl("(pick-up a)", "(stack a b)", "(pick-up c)", "(stack c a)")
trajectory = PDDL.simulate(blocksworld, bw_state, plan)
sol = PathSearchSolution(:success, plan, trajectory)
sol = EpsilonGreedyPolicy(blocksworld, sol, 0.1)

@test get_action(sol, bw_state) in bw_init_actions
@test rand_action(sol, bw_state) in bw_init_actions
@test best_action(sol, bw_state) == pddl"(pick-up a)"

probs = Dict(a => 0.1 / length(bw_init_actions) for a in bw_init_actions)
probs[pddl"(pick-up a)"] += 0.9
@test all(probs[a] ≈ p for (a, p) in get_action_probs(sol, bw_state))
act_prob = probs[pddl"(pick-up a)"]
@test get_action_prob(sol, bw_state, pddl"(pick-up a)") ≈ act_prob
@test get_action_prob(sol, bw_state, pddl"(pick-up z)") == 0.0

@test has_values(sol) == false

@test copy(sol) == sol

end

@testset "Epsilon Mixture Policy" begin

planner = AStarPlanner(HMax())
heuristic = PlannerHeuristic(planner)
sol = HeuristicVPolicy(heuristic, blocksworld, bw_spec)

sol = EpsilonMixturePolicy(blocksworld, sol, [0.1, 0.2])
@test get_mixture_weights(sol) == [0.5, 0.5]
sol = EpsilonMixturePolicy(blocksworld, sol, [0.1, 0.2], [0.4, 0.6])
@test get_mixture_weights(sol) == [0.4, 0.6]

@test get_action(sol, bw_state) in bw_init_actions
@test rand_action(sol, bw_state) in bw_init_actions
@test best_action(sol, bw_state) == pddl"(pick-up a)"

@test get_value(sol, bw_state) == -4.0
@test get_value(sol, bw_state, pddl"(pick-up b)") == -6.0
@test Dict(get_action_values(sol, bw_state)) == bw_init_q

probs_1 = Dict(a => 0.1 / length(bw_init_actions) for a in bw_init_actions)
probs_1[pddl"(pick-up a)"] += 0.9
probs_2 = Dict(a => 0.2 / length(bw_init_actions) for a in bw_init_actions)
probs_2[pddl"(pick-up a)"] += 0.8
probs = Dict(a => 0.4 * probs_1[a] + 0.6 * probs_2[a] for a in bw_init_actions)
@test all(probs[a] ≈ p for (a, p) in get_action_probs(sol, bw_state))
act_prob = probs[pddl"(pick-up a)"]
@test get_action_prob(sol, bw_state, pddl"(pick-up a)") ≈ act_prob
@test get_action_prob(sol, bw_state, pddl"(pick-up z)") == 0.0

act = pddl"(pick-up a)"
new_weights = [0.4 * probs_1[act], 0.6 * probs_2[act]]
new_weights = new_weights ./ sum(new_weights)
@test get_mixture_weights(sol, bw_state, act) ≈ new_weights

sol = TabularPolicy()
sol.V[hash(bw_state)] = bw_init_v
sol.Q[hash(bw_state)] = copy(bw_init_q)
sol.Q[hash(bw_state)][pddl"(pick-up a)"] = -8.0
sol = EpsilonMixturePolicy(blocksworld, sol, [0.0, 0.2])
probs = Dict(a => a == pddl"(pick-up a)" ? 0.1 / 3 : 0.45 + 0.1 / 3
             for a in bw_init_actions)
@test all(probs[a] ≈ p for (a, p) in get_action_probs(sol, bw_state))
@test get_action_prob(sol, bw_state, pddl"(pick-up a)") ≈ 0.1 / 3
@test get_action_prob(sol, bw_state, pddl"(pick-up b)") ≈ 0.45 + 0.1 / 3
@test get_action_prob(sol, bw_state, pddl"(pick-up c)") ≈ 0.45 + 0.1 / 3
new_weights = [0.5, (0.4 + 0.2 / 3)]
new_weights = new_weights ./ sum(new_weights)
@test get_mixture_weights(sol, bw_state, pddl"(pick-up b)") ≈ new_weights

@test has_values(sol) == true

plan = @pddl("(pick-up a)", "(stack a b)", "(pick-up c)", "(stack c a)")
trajectory = PDDL.simulate(blocksworld, bw_state, plan)
sol = PathSearchSolution(:success, plan, trajectory)
sol = EpsilonMixturePolicy(blocksworld, sol, [0.0, 0.2])

@test get_action(sol, bw_state) in bw_init_actions
@test rand_action(sol, bw_state) in bw_init_actions
@test best_action(sol, bw_state) == pddl"(pick-up a)"

probs_1 = Dict(a => 0.0 / length(bw_init_actions) for a in bw_init_actions)
probs_1[pddl"(pick-up a)"] += 1.0
probs_2 = Dict(a => 0.2 / length(bw_init_actions) for a in bw_init_actions)
probs_2[pddl"(pick-up a)"] += 0.8
probs = Dict(a => 0.5 * probs_1[a] + 0.5 * probs_2[a] for a in bw_init_actions)
@test all(probs[a] ≈ p for (a, p) in get_action_probs(sol, bw_state))
act_prob = probs[pddl"(pick-up a)"]
@test get_action_prob(sol, bw_state, pddl"(pick-up a)") ≈ act_prob
@test get_action_prob(sol, bw_state, pddl"(pick-up z)") == 0.0

act = pddl"(pick-up a)"
new_weights = [0.5 * probs_1[act], 0.5 * probs_2[act]]
new_weights = new_weights ./ sum(new_weights)
@test get_mixture_weights(sol, bw_state, act) ≈ new_weights

@test has_values(sol) == false

@test copy(sol) == sol

end

@testset "Mixture Policy" begin

planner = AStarPlanner(HMax())
heuristic = PlannerHeuristic(planner)
sol = HeuristicVPolicy(heuristic, blocksworld, bw_spec)
sol_1 = EpsilonGreedyPolicy(blocksworld, sol, 0.1)
sol_2 = BoltzmannPolicy(sol, 1.0)

sol = MixturePolicy([sol_1, sol_2])
@test get_mixture_weights(sol) == [0.5, 0.5]
sol = MixturePolicy((sol_1, sol_2), [0.4, 0.6])
@test get_mixture_weights(sol) == [0.4, 0.6]

@test get_action(sol, bw_state) in bw_init_actions
@test rand_action(sol, bw_state) in bw_init_actions

e_probs = Dict(a => 0.1 / length(bw_init_actions) for a in bw_init_actions)
e_probs[pddl"(pick-up a)"] += 0.9
b_probs = SymbolicPlanners.softmax(collect(values(bw_init_q)))
b_probs = Dict(zip(keys(bw_init_q), b_probs))
probs = Dict(a => e_probs[a] * 0.4 + b_probs[a] * 0.6 for a in bw_init_actions)
@test all(probs[a] ≈ p for (a, p) in get_action_probs(sol, bw_state))
act_prob = probs[pddl"(pick-up a)"]
@test get_action_prob(sol, bw_state, pddl"(pick-up a)") ≈ act_prob
@test get_action_prob(sol, bw_state, pddl"(pick-up z)") == 0.0

act = pddl"(pick-up a)"
new_weights = [0.4 * e_probs[act], 0.6 * b_probs[act]]
new_weights = new_weights ./ sum(new_weights)
@test get_mixture_weights(sol, bw_state, act) ≈ new_weights

@test has_values(sol) == false

@test copy(sol) == sol

end

@testset "Multi-Solution" begin

planner = AStarPlanner(HMax())
heuristic = PlannerHeuristic(planner)
sol = HeuristicVPolicy(heuristic, blocksworld, bw_spec)
sol_1 = EpsilonGreedyPolicy(blocksworld, sol, 0.1)
sol_2 = BoltzmannPolicy(sol, 1.0)

sol = MultiSolution(sol_1, sol_2)

@test get_action(sol, bw_state) in bw_init_actions
@test rand_action(sol, bw_state) in bw_init_actions

@test best_action(sol, bw_state) == best_action(sol_1, bw_state)
@test best_action(sol, bw_state) == pddl"(pick-up a)"

@test get_action_probs(sol, bw_state) == get_action_probs(sol_1, bw_state)
@test get_action_probs(sol, bw_state) != get_action_probs(sol_2, bw_state)
@test get_action_prob(sol, bw_state, pddl"(pick-up a)") ==
    get_action_prob(sol_1, bw_state, pddl"(pick-up a)")
@test get_action_prob(sol, bw_state, pddl"(pick-up a)") !=
    get_action_prob(sol_2, bw_state, pddl"(pick-up a)")

@test has_values(sol) == true

@test copy(sol) == sol

bw_selector(solutions) = solutions[1]
bw_selector(solutions, state) =
    state[pddl"(holding a)"] ? solutions[2] : solutions[1]
sol = MultiSolution((sol_1, sol_2), bw_selector)

@test get_action(sol, bw_state) in bw_init_actions
@test rand_action(sol, bw_state) in bw_init_actions

@test best_action(sol, bw_state) == best_action(sol_1, bw_state)
@test best_action(sol, bw_state) == pddl"(pick-up a)"

@test get_action_probs(sol, bw_state) == get_action_probs(sol_1, bw_state)
@test get_action_probs(sol, bw_state) != get_action_probs(sol_2, bw_state)
@test get_action_prob(sol, bw_state, pddl"(pick-up a)") ==
    get_action_prob(sol_1, bw_state, pddl"(pick-up a)")
@test get_action_prob(sol, bw_state, pddl"(pick-up a)") !=
    get_action_prob(sol_2, bw_state, pddl"(pick-up a)")

next_bw_state = transition(blocksworld, bw_state, pddl"(pick-up a)")

@test best_action(sol, next_bw_state) == best_action(sol_2, next_bw_state)
@test best_action(sol, next_bw_state) == pddl"(stack a b)"

@test get_action_probs(sol, next_bw_state) != get_action_probs(sol_1, next_bw_state)
@test get_action_probs(sol, next_bw_state) == get_action_probs(sol_2, next_bw_state)
@test get_action_prob(sol, next_bw_state, pddl"(stack a b)") !=
    get_action_prob(sol_1, next_bw_state, pddl"(stack a b)")
@test get_action_prob(sol, next_bw_state, pddl"(stack a b)") ==
    get_action_prob(sol_2, next_bw_state, pddl"(stack a b)")

@test has_values(sol) == true

@test copy(sol) == sol

end

end
