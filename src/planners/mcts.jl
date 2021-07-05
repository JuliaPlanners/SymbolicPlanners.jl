export MonteCarloTreeSearch, MCTS

"Planner that uses Monte Carlo Tree Search (MCTS)."
@kwdef mutable struct MonteCarloTreeSearch <: Planner
	heuristic::Heuristic = GoalCountHeuristic()
	explore_noise::Float64 = 2.0
	n_rollouts::Int64 = 50
	max_depth::Int64 = 50
end

const MCTS = MonteCarloTreeSearch

"Policy solution where values and q-values are directly stored in a hashtable."
@kwdef mutable struct PolicyTreeSolution <: PolicySolution
	Q::Dict{UInt64,Dict{Term,Float64}} = Dict()
	state_visits::Dict{UInt64,Int} = Dict()
	action_visits::Dict{UInt64,Dict{Term,Float64}} = Dict()
end

best_action(sol::PolicyTreeSolution, state::State) =
    argmax(sol.Q[hash(state)])

rand_action(sol::PolicyTreeSolution, state::State) =
	best_action(sol, state)

function solve(planner::MonteCarloTreeSearch,
			   domain::Domain, state::State, spec::Specification)
	@unpack n_rollouts, max_depth = planner
	@unpack heuristic, explore_noise = planner
	discount = get_discount(spec)
    # Initialize solution
	sol = PolicyTreeSolution()
	state_id = hash(state)
	h_val = -heuristic(domain, state, spec) + 1
	actions = available(state, domain)
	sol.state_visits[state_id] = 0
	sol.action_visits[state_id] = Dict{Term,Int}(a => 0 for a in actions)
	sol.Q[state_id] = Dict{Term,Float64}(a => h_val for a in actions)
	# Perform rollouts from initial state
	initial_state = state
    a_visited, s_visited = Term[], UInt64[]
	for n in 1:n_rollouts
		state, state_id = initial_state, hash(initial_state)
		rollout_val = 0.0
        # Rollout until maximum depth
        for t in 1:max_depth
			# Terminate if rollout reaches goal
            if is_goal(spec, domain, state)
				rollout_val = 1.0 * discount^t
				break # TODO : Handle general goal rewards
			end
			# Select action
			act = ucb_selection(sol, state, domain, explore_noise)
			push!(s_visited, state_id)
			push!(a_visited, act)
			# Transition to next state
            state = transition(domain, state, act)
			state_id = hash(state)
			# Insert search node
			if !(state_id in keys(sol.state_visits))
				h_val = -heuristic(domain, state, spec) + 1
				actions = available(state, domain)
				sol.Q[state_id] =
					Dict{Term,Float64}(a => h_val for a in actions)
				sol.action_visits[state_id] =
					Dict{Term,Int}(a => 0 for a in actions)
				sol.state_visits[state_id] = 0
				rollout_val =
					rollout_estimator(domain, state, spec, max_depth-t)
				if (discount < 1) rollout_val *= discount^t end
				break
			end
        end
        # Backpropagate value estimates
		q_val = rollout_val
        while length(s_visited) > 0
            state_id = pop!(s_visited)
			act = pop!(a_visited)
			sol.state_visits[state_id] += 1
			sol.action_visits[state_id][act] += 1
			act_cost = 0 # TODO: Handle general action costs
			q_val = discount * rollout_val - act_cost
			sol.Q[state_id][act] +=
				(q_val-sol.Q[state_id][act]) / sol.action_visits[state_id][act]
        end
	end
	return sol
end

solve(planner::MCTS, domain::Domain, state::State, goals::Vector{<:Term}) =
    solve(planner, domain, state, GoalReward(goals))
solve(planner::MCTS, domain::Domain, state::State, goal::Term) =
    solve(planner, domain, state, GoalReward(goal))

function ucb_selection(sol, state, domain, c)
	actions = available(state, domain)
	state_id = hash(state)
	state_visits = sol.state_visits[state_id]
	ucb_vals = map(actions) do act
		act_visits = sol.action_visits[state_id][act]
		q_val = sol.Q[state_id][act]
		if c == 0 || state_visits == 0 || (state_visits == 1 && act_visits == 0)
			val = q_val
		else
			val = q_val + c*sqrt(log(state_visits) / act_visits)
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
