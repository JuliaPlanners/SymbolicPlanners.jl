## Basic heuristics ##
export NullHeuristic, GoalCountHeuristic

"""
    NullHeuristic()

Null heuristic that always returns zero.
"""
struct NullHeuristic <: Heuristic end

compute(h::NullHeuristic, domain::Domain, state::State, spec::Specification) = 0

"""
    GoalCountHeuristic(dir=:forward)

Heuristic that counts the number of goals un/satisfied. Can be used in either
the `:forward` or `:backward` direction. The latter should be used for search
with [`BackwardPlanner`](@ref).
"""
struct GoalCountHeuristic <: Heuristic
    dir::Symbol # :forward or :backward
    GoalCountHeuristic() = new(:forward)
    GoalCountHeuristic(dir) = new(dir)
end

function compute(h::GoalCountHeuristic,
                 domain::Domain, state::State, spec::Specification)
    goals = get_goal_terms(spec)
    count = sum(!satisfy(domain, state, g) for g in goals)
    return h.dir == :backward ? length(goals) - count : count
end
