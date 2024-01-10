using Plots, Graphs, GraphPlot
using Cairo, Compose

export landmark_graph_draw_png
export landmark_graph_print

function landmark_graph_draw_png(filename::String, landmark_graph::LandmarkGraph, planning_graph::PlanningGraph)
    node_lookup::Dict{LandmarkNode, Int} = Dict(map(reverse, enumerate(landmark_graph.nodes)))
    ne::Int = 0
    nn::Int = 0
    names::Vector{String} = []
    fadj::Vector{Vector{Int}} = []
    badj::Vector{Vector{Int}} = []
    edgelabels::Vector{String} = []
    for node::LandmarkNode in landmark_graph.nodes
        nn += 1
        push!(names, "$(factpair_to_term(first(node.landmark.facts), planning_graph))")
        children::Vector{Int} = []
        for child::Pair{LandmarkNode, EdgeType} in node.children
            ne += 1
            j::Int = node_lookup[child.first]
            push!(edgelabels, "$(child.second)")
            push!(children, j)
        end
        push!(fadj, children)
        parents::Vector{Int} = []
        for parent::Pair{LandmarkNode, EdgeType} in node.parents
            j::Int = node_lookup[parent.first]
            push!(parents, j)
        end
        push!(badj, parents)
    end
    g = SimpleDiGraph(ne, fadj, badj)

    layout=(args...)->spring_layout(args...; C=15)
    p = gplot(g, layout=layout, nodelabel=names, NODELABELSIZE=3.0, NODESIZE=0.06, EDGELABELSIZE=3.0,
                edgelabel=edgelabels, arrowlengthfrac=0.02, edgelabelc=colorant"orange")

    draw(PNG(filename, 32cm, 32cm), p)
end

function landmark_graph_print(landmark_graph::LandmarkGraph, planning_graph::PlanningGraph)
    node_lookup::Dict{LandmarkNode, Int} = Dict(map(reverse, enumerate(landmark_graph.nodes)))

    for node::Tuple{Int, LandmarkNode} in enumerate(landmark_graph.nodes)
        println("Landmark $(node[1]) ($(factpair_to_term(first(node[2].landmark.facts), planning_graph)))")
        for child::Pair{LandmarkNode, EdgeType} in node[2].children
            println("    Edge to $(node_lookup[child.first]) - $(child.second)")
        end
    end
    println()
end

function factpair_to_term(factpair::FactPair, planning_graph::PlanningGraph) :: Term
    effect::Term = planning_graph.conditions[factpair.var]
    if factpair.value == 0
        effect = Compound(:not, [effect])
    end
    return effect
end
