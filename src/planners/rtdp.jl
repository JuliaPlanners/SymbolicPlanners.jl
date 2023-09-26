export RealTimeDynamicPlanner, RTDP

"""
    RealTimeDynamicPlanner(;
        heuristic::Heuristic = GoalCountHeuristic(),
        n_rollouts::Int = 50,
        max_depth::Int = 50,
        rollout_noise::Float64 = 0.0,
        action_noise::Float64 = 0.0,
        verbose::Bool = false,
        callback = verbose ? LoggerCallback() : nothing
    )

Planner that uses Real Time Dynamic Programming (`RTDP` for short), a form
of asynchronous value iteration which performs greedy rollouts from the initial
state, updating the value estimates of states encountered along the way [1].

If a `heuristic` is provided, the negated heuristic value will be used as an
initial value estimate for newly encountered states (since the value of a state
in a shortest path problem is the cost to reach the goal), thereby guiding
early rollouts.

For admissible (i.e. optimistic) heuristics, convergence to the true value
function is guaranteed in the reachable state space after a sufficient number of
rollouts.

Returns a [`TabularPolicy`](@ref) (wrapped in a [`BoltzmannPolicy`](@ref) if
`action_noise > 0`), which stores the value estimates and action Q-values for
each encountered state. 

[1] A. G. Barto, S. J. Bradtke, and S. P. Singh, "Learning to Act using
Real-Time Dynamic Programming," Artificial Intelligence, vol. 72, no. 1,
pp. 81–138, Jan. 1995, <https://doi.org/10.1016/0004-3702(94)00011-O>.

# Arguments

$(FIELDS)
"""
@kwdef mutable struct RealTimeDynamicPlanner <: Planner
    "Search heuristic used to initialize the value function."
    heuristic::Heuristic = GoalCountHeuristic()
    "Number of rollouts to perform from the initial state."
    n_rollouts::Int = 50
    "Maximum depth of each rollout."
    max_depth::Int = 50
    "Amount of Boltzmann noise during simulated rollouts."
    rollout_noise::Float64 = 0.0
    "Amount of Boltzmann action noise for the returned policy."
    action_noise::Float64 = 0.0
    "Flag to print debug information during search."
    verbose::Bool = false
    "Callback function for logging, etc."
    callback::Union{Nothing, Function} = verbose ? LoggerCallback() : nothing
end

@auto_hash RealTimeDynamicPlanner
@auto_equals RealTimeDynamicPlanner

const RTDP = RealTimeDynamicPlanner

function Base.copy(p::RealTimeDynamicPlanner)
    return RealTimeDynamicPlanner(p.heuristic, p.n_rollouts, p.max_depth,
                                  p.rollout_noise, p.action_noise,
                                  p.verbose, p.callback)
end

function solve(planner::RealTimeDynamicPlanner,
               domain::Domain, state::State, spec::Specification)
    # Simplify goal specification
    spec = simplify_goal(spec, domain, state)
    # Precompute heuristic information
    precompute!(planner.heuristic, domain, state, spec)
    # Initialize then refine solution
    default = FunctionalVPolicy(planner.heuristic, domain, spec)
    sol = TabularPolicy(default)
    sol.V[hash(state)] = -compute(planner.heuristic, domain, state, spec)
    sol = solve!(sol, planner, domain, state, spec)
    # Wrap in Boltzmann policy if needed
    return planner.action_noise == 0 ?
        sol : BoltzmannPolicy(sol, planner.action_noise)
end

function solve!(sol::TabularPolicy, planner::RealTimeDynamicPlanner,
                domain::Domain, state::State, spec::Specification)
    @unpack heuristic, action_noise = planner
    @unpack n_rollouts, max_depth, rollout_noise = planner
    # Simplify goal specification
    spec = simplify_goal(spec, domain, state)
    # Precompute heuristic information
    ensure_precomputed!(heuristic, domain, state, spec)
    # Run callback
    if !isnothing(planner.callback)
        planner.callback(planner, sol, 0, [state], :init)
    end
    # Perform rollouts from initial state
    initial_state = state
    ro_policy = rollout_noise == 0 ? sol : BoltzmannPolicy(sol, rollout_noise)
    state_history = Vector{typeof(state)}()
    for n in 1:n_rollouts
        state = initial_state
        act = PDDL.no_op
        stop_reason = :max_depth
        # Rollout until maximum depth
        for t in 1:max_depth
            push!(state_history, state)
            if is_goal(spec, domain, state, act) # Stop if goal is reached
                stop_reason = :goal_reached
                break
            end
            update_values!(planner, sol, domain, state, stop_reason, spec)
            act = get_action(ro_policy, state)
            if ismissing(act) # Stop if we hit a dead-end
                stop_reason = :dead_end
                break
            end 
            state = transition(domain, state, act)
        end
        # Run callback
        if !isnothing(planner.callback)
            planner.callback(planner, sol, n, state_history, stop_reason)
        end
        # Post-rollout update
        while length(state_history) > 0
            state = pop!(state_history)
            update_values!(planner, sol, domain, state, stop_reason, spec)
            stop_reason = :none
        end
    end
    return sol
end

function solve!(sol::BoltzmannPolicy{TabularPolicy},
                planner::RealTimeDynamicPlanner,
                domain::Domain, state::State, spec::Specification)
    sol = solve!(planner, sol.policy, domain, state, spec)
    return BoltzmannPolicy(sol, planner.action_noise)
end

function update_values!(planner::RealTimeDynamicPlanner, sol::TabularPolicy,
                        domain::Domain, state::State,
                        stop_reason::Symbol, spec::Specification)
    @unpack action_noise = planner
    state_id = hash(state)
    qs = get!(Dict{Term,Float64}, sol.Q, state_id)
    # Back-propagate values from successor states
    for act in available(domain, state)
        next_state = transition(domain, state, act)
        next_v = get!(sol.V, hash(next_state)) do
            -compute(planner.heuristic, domain, next_state, spec)
        end
        if has_action_goal(spec) && is_goal(spec, domain, next_state, act)
            next_v = 0.0
        end
        r = get_reward(spec, domain, state, act, next_state)
        qs[act] =  get_discount(spec) * next_v + r
    end
    if stop_reason == :goal_reached && !has_action_goal(spec)
        sol.V[state_id] = 0.0
    elseif stop_reason == :dead_end
        sol.V[state_id] = -Inf
    elseif action_noise == 0
        sol.V[state_id] = maximum(values(qs))
    else
        qvals = collect(values(qs))
        sol.V[state_id] = sum(softmax(qvals ./ action_noise) .* qvals)
    end
    return nothing
end

function refine!(sol::PolicySolution, planner::RealTimeDynamicPlanner,
                 domain::Domain, state::State, spec::Specification)
    return solve!(sol, planner, domain, state, spec)
end

function (cb::LoggerCallback)(
    planner::RealTimeDynamicPlanner,
    sol::PolicySolution, n::Int, visited, stop_reason::Symbol
)
    if n == 0 && get(cb.options, :log_header, true)
        @logmsg cb.loglevel "Running RTDP..."
        n_rollouts, max_depth = planner.n_rollouts, planner.max_depth
        @logmsg cb.loglevel "n_rollouts = $n_rollouts, max_depth = $max_depth"
        rollout_noise, action_noise = planner.rollout_noise, planner.action_noise
        @logmsg cb.loglevel "rollout_noise = $rollout_noise, action_noise = $action_noise"
    end
    log_period = get(cb.options, :log_period, 1)
    if n > 0 && n % log_period == 0
        depth = length(visited)
        start_v = get_value(sol, visited[1])
        stop_v = get_value(sol, visited[end]) 
        @logmsg(cb.loglevel, 
                "Rollout $n: depth = $depth, stop reason = $stop_reason, " * 
                "start value = $start_v, stop value = $stop_v")
    end
    return nothing
end
