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

function compute(h::GoalCountHeuristic,
                 domain::Domain, state::State, spec::ActionGoal)
    @assert h.dir == :forward "Backwards goal count unsupported for ActionGoal."
    # Convert action precondition to goals
    goals = PDDL.flatten_conjs(PDDL.get_precond(domain, spec.action))
    # Filter out global predicates with global function subterms
    goals = filter!(goals) do g
        if PDDL.is_global_pred(g) && g isa Compound
            return all(!PDDL.has_global_func(a) for a in g.args)
        else
            return true
        end
    end
    # Add constraint predicates to goals if unaffected by action
    affected = PDDL.get_affected(PDDL.get_action(domain, spec.action.name))
    for constraint in spec.constraints
        PDDL.has_name(constraint, affected) && continue
        PDDL.has_global_func(constraint) && continue
        push!(goals, constraint)
    end        
    count = sum(!satisfy(domain, state, g) for g in goals)
    return count + spec.step_cost
end
