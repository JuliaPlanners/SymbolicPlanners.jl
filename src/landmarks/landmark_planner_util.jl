export remove_reasonable_edges
export remove_natural_edges
export landmark_to_terms
export get_sources

"""
Utility Functions for both the landmark based Planners, LM Local and LM Local Smart.
"""

function remove_reasonable_edges(lm_graph::LandmarkGraph)
    for lm in lm_graph.nodes
        for (child, edge) in lm.children
            if edge == REASONABLE || edge == NATURAL delete!(lm.children, child) end
        end
    end
end

function remove_natural_edges(lm_graph::LandmarkGraph)
    for lm in lm_graph.nodes
        for (parent, edge) in lm.parents
            if edge == REASONABLE || edge == NATURAL delete!(lm.parents, parent) end
        end
    end
end

function get_sources(lm_graph::LandmarkGraph) :: Set{LandmarkNode}
    res::Set{LandmarkNode} = Set()
    for lm in lm_graph.nodes
        if length(lm.parents) == 0 push!(res, lm) end
    end
    return res
end

function landmark_to_terms(lm::Landmark, p_graph::PlanningGraph) :: Term
    res::Vector{Term} = Vector()
    for fact_p :: FactPair in lm.facts
        if fact_p.value == 1
            push!(res, p_graph.conditions[fact_p.var])
        else 
            push!(res, Compound(:not, [p_graph.conditions[fact_p.var]]))
        end
    end
    if length(res) > 1 return Compound(:and, res)
    elseif length(res) == 1 return first(res)
    else return Compound(:and, []) end
end
