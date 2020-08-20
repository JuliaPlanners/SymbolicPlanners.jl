## Basic heuristics ##
export NullHeuristic, GoalCountHeuristic, ManhattanHeuristic

"Null heuristic that always returns zero."
struct NullHeuristic <: Heuristic end

Base.hash(::NullHeuristic, h::UInt) = hash(NullHeuristic, h)

compute(h::NullHeuristic, domain::Domain, state::State, goal_spec::GoalSpec) = 0

"Heuristic that counts the number of goals un/satisfied."
struct GoalCountHeuristic <: Heuristic
    dir::Symbol # :forward or :backward
    GoalCountHeuristic() = new(:forward)
    GoalCountHeuristic(dir) = new(dir)
end

Base.hash(::GoalCountHeuristic, h::UInt) = hash(GoalCountHeuristic, h)

function compute(heuristic::GoalCountHeuristic,
                 domain::Domain, state::State, goal_spec::GoalSpec)
    count = sum([!state[domain, g] for g in goal_spec.goals])
    return heuristic.dir == :backward ? length(goal_spec.goals) - count : count
end

"Computes Manhattan distance to the goal for the specified numeric fluents."
mutable struct ManhattanHeuristic <: Heuristic
    fluents::Vector{Term}
    goal_state::State
    ManhattanHeuristic(fluents) = new(fluents)
    ManhattanHeuristic(fluents, goal_state) = new(fluents, goal_state)
end

Base.hash(heuristic::ManhattanHeuristic, h::UInt) =
    hash(heuristic.fluents, hash(ManhattanHeuristic, h))

function precompute!(heuristic::ManhattanHeuristic,
                     domain::Domain, state::State, goal_spec::GoalSpec)
    heuristic.goal_state = State(goal_spec.goals)
    return heuristic
end

function compute(heuristic::ManhattanHeuristic,
                 domain::Domain, state::State, goal_spec::GoalSpec)
    # Precompute if necessary
    if !isdefined(heuristic, :goal_state)
        precompute!(heuristic, domain, state, goal_spec) end
    @unpack fluents, goal_state = heuristic
    goal_vals = [goal_state[domain, f] for f in fluents]
    curr_vals = [state[domain, f] for f in fluents]
    dist = sum(abs.(goal_vals - curr_vals))
    return dist
end
