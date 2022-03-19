## Basic heuristics ##
export NullHeuristic, GoalCountHeuristic, ManhattanHeuristic, PlannerHeuristic

"Null heuristic that always returns zero."
struct NullHeuristic <: Heuristic end

Base.hash(::NullHeuristic, h::UInt) = hash(NullHeuristic, h)

compute(h::NullHeuristic, domain::Domain, state::State, spec::Specification) = 0

"Heuristic that counts the number of goals un/satisfied."
struct GoalCountHeuristic <: Heuristic
    dir::Symbol # :forward or :backward
    GoalCountHeuristic() = new(:forward)
    GoalCountHeuristic(dir) = new(dir)
end

Base.hash(heuristic::GoalCountHeuristic, h::UInt) =
    hash(heuristic.dir, hash(GoalCountHeuristic, h))

function compute(h::GoalCountHeuristic,
                 domain::Domain, state::State, spec::Specification)
    goals = get_goal_terms(spec)
    count = sum(!satisfy(domain, state, g) for g in goals)
    return h.dir == :backward ? length(goals) - count : count
end

"Computes Manhattan distance to the goal for the specified numeric fluents."
mutable struct ManhattanHeuristic <: Heuristic
    fluents::Vector{Term}
    goal::State
    pre_key::UInt64 # Precomputation hash
    ManhattanHeuristic(fluents) = new(fluents)
end

Base.hash(heuristic::ManhattanHeuristic, h::UInt) =
    hash(heuristic.fluents, hash(ManhattanHeuristic, h))

function precompute!(h::ManhattanHeuristic,
                     domain::Domain, state::State, spec::Specification)
    h.goal = goalstate(domain, PDDL.get_objtypes(state), get_goal_terms(spec))
    h.pre_key = objectid(spec)
    return h
end

function precompute!(h::ManhattanHeuristic,
                     domain::CompiledDomain, state::State, spec::Specification)
    goal = goalstate(PDDL.get_source(domain), PDDL.get_objtypes(state),
                     get_goal_terms(spec))
    h.goal = typeof(state)(goal)
    h.pre_key = objectid(spec)
    return h
end

function is_precomputed(h::ManhattanHeuristic,
                        domain::Domain, state::State, spec::Specification)
    return (isdefined(h, :goal) && objectid(spec) == h.pre_key)
end

function compute(h::ManhattanHeuristic,
                 domain::Domain, state::State, spec::Specification)
    # Precompute if necessary
    if !is_precomputed(h, domain, state, spec)
        precompute!(h, domain, state, spec) end
    # Compute L1 distance between fluents in the current and goal state
    dist = sum(abs(evaluate(domain, h.goal, f) - evaluate(domain, state, f))
               for f in h.fluents)
    return dist
end

"Computes distance to the goal based on planner solution."
mutable struct PlannerHeuristic{P <: Planner} <: Heuristic
    planner::P
end

function Base.hash(heuristic::PlannerHeuristic{P}, h::UInt) where {P}
    h = hash(P, hash(PlannerHeuristic, h))
    for f in fieldnames(P)
        h = hash(getfield(heuristic.planner, f), h)
    end
    return h
end

function compute(h::PlannerHeuristic,
                 domain::Domain, state::State, spec::Specification)
    sol = h.planner(domain, state, spec)
    if sol isa OrderedSolution
        return length(sol)
    elseif sol isa PolicySolution
        return -get_value(sol, state)
    else
        return Inf
    end
end
