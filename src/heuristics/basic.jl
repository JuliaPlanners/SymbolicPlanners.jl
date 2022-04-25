## Basic heuristics ##
export NullHeuristic, GoalCountHeuristic

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
