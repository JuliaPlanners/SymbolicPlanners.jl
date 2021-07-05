export MonteCarloTreeSearch, MCTS

"MCTS policy solution."
@kwdef struct MCTSTreeSolution <: PolicySolution
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

has_state_node(sol::MCTSTreeSolution, state::State) =
	hash(state) in keys(sol.s_visits)

"Planner that uses Monte Carlo Tree Search (MCTS)."
@kwdef mutable struct MonteCarloTreeSearch <: Planner
	heuristic::Heuristic = GoalCountHeuristic()
	n_rollouts::Int64 = 50
	max_depth::Int64 = 50
	explore_noise::Float64 = 2.0
end

const MCTS = MonteCarloTreeSearch

function solve(planner::MonteCarloTreeSearch,
			   domain::Domain, state::State, spec::Specification)
	@unpack n_rollouts, max_depth = planner
	@unpack heuristic, explore_noise = planner
	discount = get_discount(spec)
    # Initialize solution
	sol = MCTSTreeSolution()
	insert_node!(planner, sol, domain, state, spec)
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
			act = ucb_selection(sol, state, domain, explore_noise)
			push!(s_visited, state)
			push!(a_visited, act)
			# Transition to next state
            state = transition(domain, state, act)
			# Insert search node
			if !has_state_node(sol, state)
				insert_node!(planner, sol, domain, state, spec)
				value = get_discount(spec)^t *
					rollout_estimator(domain, state, spec, max_depth-t)
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
end

function ucb_selection(sol, state, domain, c)
	actions = available(state, domain)
	state_id = hash(state)
	s_visits = sol.s_visits[state_id]
	ucb_vals = map(actions) do act
		act_visits = sol.a_visits[state_id][act]
		q_val = sol.Q[state_id][act]
		if c == 0 || s_visits == 0 || (s_visits == 1 && act_visits == 0)
			val = q_val
		else
			val = q_val + c*sqrt(log(s_visits) / act_visits)
		end
		return val
	end
	probs = softmax(ucb_vals ./ 1.0)
    act = sample(actions, Weights(probs, 1.0))
	return act
end

function rollout_estimator(domain, state, spec, depth)
	sim = RewardAccumulator(depth)
	policy = RandomPolicy(domain)
	return sim(policy, domain, state, spec)
end
