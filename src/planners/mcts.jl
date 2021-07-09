export MonteCarloTreeSearch, MCTS, MCTSTreeSolution
export MCTSNodeSelector, MaxUCBSelector, BoltzmannUCBSelector

## MCTS solution type ##

"MCTS policy solution."
@kwdef mutable struct MCTSTreeSolution <: PolicySolution
	Q::Dict{UInt64,Dict{Term,Float64}} = Dict()
	s_visits::Dict{UInt64,Int} = Dict()
	a_visits::Dict{UInt64,Dict{Term,Float64}} = Dict()
end

get_action(sol::MCTSTreeSolution, state::State) =
    best_action(sol, state)
best_action(sol::MCTSTreeSolution, state::State) =
    argmax(sol.Q[hash(state)])
rand_action(sol::MCTSTreeSolution, state::State) =
    best_action(sol, state)
get_value(sol::MCTSTreeSolution, state::State) =
    maximum(sol.Q[hash(state)])
get_value(sol::MCTSTreeSolution, state::State, action::Term) =
    sol.Q[hash(state)][action]
get_action_values(sol::MCTSTreeSolution, state::State) =
    pairs(sol.Q[hash(state)])

has_state_node(sol::MCTSTreeSolution, state::State) =
	hash(state) in keys(sol.s_visits)

## Selection strategies for leaf nodes ##

"Max upper-confidence bound (UCB) action policy."
struct MaxUCBPolicy <: PolicySolution
	tree::MCTSTreeSolution
	confidence::Float64
end

function get_action_values(sol::MaxUCBPolicy, state::State)
	state_id = hash(state)
	s_visits = sol.tree.s_visits[state_id]
	vals = map(collect(sol.tree.Q[state_id])) do (act, val)
		a_visits = sol.tree.a_visits[state_id][act]
		if s_visits != 0 && !(s_visits == 1 && a_visits == 0)
			val += sol.confidence * sqrt(log(s_visits) / a_visits)
		end
		return act => val
	end
	return vals
end

function best_action(sol::MaxUCBPolicy, state::State)
    a_vals = get_action_values(sol, state)
	best_idx = argmax(last.(a_vals))
	return first(a_vals[best_idx])
end

get_action(sol::MaxUCBPolicy, state::State) =
    best_action(sol, state)
rand_action(sol::MaxUCBPolicy, state::State) =
    best_action(sol, state)
get_value(sol::MaxUCBPolicy, state::State) =
    maximum(last, get_action_values(sol, state))
get_value(sol::MaxUCBPolicy, state::State, action::Term) =
    findfirst(x -> first(x) == action, get_action_values(sol, state))

BoltzmannUCBPolicy(tree::MCTSTreeSolution, confidence, temperature) =
	BoltzmannPolicy(MaxUCBPolicy(tree, confidence), temperature)

"Abstract type for MCTS node selection strategies."
abstract type MCTSNodeSelector end

(sel::MCTSNodeSelector)(tree::MCTSTreeSolution, domain::Domain, state::State) =
	error("Not implemented.")

"Max UCB selection strategy."
@kwdef struct MaxUCBSelector <: MCTSNodeSelector
	confidence::Float64 = 2.0
end

(sel::MaxUCBSelector)(tree::MCTSTreeSolution, ::Domain, state::State) =
 	get_action(MaxUCBPolicy(tree, sel.confidence), state)

"Boltzmann UCB selection strategy."
@kwdef struct BoltzmannUCBSelector <: MCTSNodeSelector
	confidence::Float64 = 2.0
	temperature::Float64 = 1.0
end

(sel::BoltzmannUCBSelector)(tree::MCTSTreeSolution, ::Domain, state::State) =
 	get_action(BoltzmannUCBPolicy(tree, sel.confidence, sel.temperature), state)

## Leaf node estimators ##

"Abstract type for MCTS leaf value estimators."
abstract type MCTSLeafEstimator end

(e::MCTSLeafEstimator)(::Domain, ::State, ::Specification, depth::Int) =
	error("Not implemented.")

"Estimates value as a constant."
struct ConstantEstimator{C <: Real} <: MCTSLeafEstimator
	value::C
end

(e::ConstantEstimator)(::Domain, ::State, ::Specification, ::Int) = e.value

"Estimates value via uniform random rollouts."
struct RandomRolloutEstimator <: MCTSLeafEstimator end

function (e::RandomRolloutEstimator)(domain::Domain, state::State,
								     spec::Specification, depth::Int)
	sim = RewardAccumulator(depth)
	policy = RandomPolicy(domain)
	return sim(policy, domain, state, spec)
end

"Estimates value via policy rollouts."
struct PolicyRolloutEstimator{P <: PolicySolution} <: MCTSLeafEstimator
	policy::P
end

function (e::PolicyRolloutEstimator)(domain::Domain, state::State,
									 spec::Specification, depth::Int)
	sim = RewardAccumulator(depth)
	return sim(e.policy, domain, state, spec)
end

## Main algorithm ##

"Planner that uses Monte Carlo Tree Search (MCTS)."
@kwdef mutable struct MonteCarloTreeSearch{S,E} <: Planner
	n_rollouts::Int64 = 50
	max_depth::Int64 = 50
	heuristic::Heuristic = NullHeuristic() # Initial value heuristic
	selector::S = BoltzmannUCBSelector() # Node selection strategy
	estimator::E = RandomRolloutEstimator() # Leaf node value estimator
end

const MCTS = MonteCarloTreeSearch

function solve(planner::MonteCarloTreeSearch,
			   domain::Domain, state::State, spec::Specification)
	@unpack n_rollouts, max_depth = planner
	@unpack heuristic, selector, estimator = planner
	discount = get_discount(spec)
	# Precompute heuristic information
    precompute!(heuristic, domain, state, spec)
    # Initialize solution
	sol = MCTSTreeSolution()
	sol = insert_node!(planner, sol, domain, state, spec)
	# Perform rollouts from initial state
	a_visited, s_visited = Term[], State[]
	initial_state = state
	for n in 1:n_rollouts
		state = initial_state
		value = 0.0
        # Rollout until maximum depth
        for t in 1:max_depth
			# Terminate if rollout reaches goal
            if is_goal(spec, domain, state) break end
			# Select action
			act = selector(sol, domain, state)
			push!(s_visited, state)
			push!(a_visited, act)
			# Transition to next state
            state = transition(domain, state, act)
			# Insert leaf node and evaluate
			if !has_state_node(sol, state)
				insert_node!(planner, sol, domain, state, spec)
				value = get_discount(spec)^t *
						estimator(domain, state, spec, max_depth-t)
				break
			end
        end
        # Backpropagate value estimates
		next_state = state
        while length(s_visited) > 0
            state, act = pop!(s_visited), pop!(a_visited)
			state_id = hash(state)
			# Update visitation counts
			sol.s_visits[state_id] += 1
			sol.a_visits[state_id][act] += 1
			# Compute value (accumulated discounted reward)
			reward = get_reward(spec, domain, state, act, next_state)
			value = get_discount(spec) * value + reward
			# Take weighted average with existing Q value
			sol.Q[state_id][act] +=
				(value-sol.Q[state_id][act]) / sol.a_visits[state_id][act]
			state = next_state
        end
	end
	return sol
end

function insert_node!(planner::MonteCarloTreeSearch, sol::MCTSTreeSolution,
			   		  domain::Domain, state::State, spec::Specification)
	actions = available(state, domain)
	qs = map(actions) do act
        next_state = transition(domain, state, act)
        r = get_reward(spec, domain, state, act, next_state)
        h_val = planner.heuristic(domain, next_state, spec)
        return get_discount(spec) * (-h_val) + r
    end
	state_id = hash(state)
    sol.Q[state_id] = Dict{Term,Float64}(zip(actions, qs))
	sol.a_visits[state_id] = Dict{Term,Int}(a => 0 for a in actions)
	sol.s_visits[state_id] = 0
	return sol
end
