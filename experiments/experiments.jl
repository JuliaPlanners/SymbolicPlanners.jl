## Benchmarking experiments for planners on propositional domains ##
using PDDL, SymbolicPlanners, Test, PlanningDomains
using DataFrames, CSV, Dates, Statistics


planners = ["GoalCount", "HAdd", "LM_Count", "LM_Local-HAdd", "LM_Local_Smart-HAdd"]
benchmark_file = "ordered-landmarks-benchmark.txt"

# Parse benchmark file
domains::Vector{Pair{String, String}} = []
instances::Dict{String, Vector{Pair{String, String}}} = Dict()

function parse_benchmark(benchmark_file_path::String)
    domain_string::String = ""
    domain_path::String = ""
    lines::Vector{String} = readlines(joinpath(@__DIR__, benchmark_file_path))
    for s::String in lines
        if s[1:3] == " - " && !isempty(domain_string)
            s_parts::Vector{String} = split(s[3:end], '=')
            if length(s_parts) == 2 && !(' ' in strip(s_parts[1])) && '/' in s_parts[2]
                instance_string::String = strip(s_parts[1])
                instance_path::String = strip(s_parts[2])
                if isempty(domain_path)
                    parts::Vector{String} = split(instance_path, '/')
                    domain_path = joinpath(@__DIR__, joinpath(parts[1:length(parts) - 1]), "domain.pddl")
                end

                if !haskey(instances, domain_string)
                    push!(domains, Pair(domain_string, domain_path))
                    instances[domain_string] = []
                end
                push!(instances[domain_string], Pair(instance_string, joinpath(@__DIR__, instance_path)))
            end
        elseif !(' ' in strip(s))
            domain_string = strip(s)
            domain_path = ""
        end
    end
end
parse_benchmark(benchmark_file)

TIMEOUT = 180.0
NRUNS = 3

# Store results in data frame
df = DataFrame(domain=String[], problem=String[], problem_size=Int[],
               compiled=Bool[], planner=String[], run=Int[],
               n_steps=Int[], time=Float64[], bytes=Int[],
               n_eval=Int[], n_expand=Int[], n_landmarks=Int[], correct_goal=Bool[])

#JuliaPlannerRepo domains
for (d_name::String, d_path::String) in domains
    domain = load_domain(d_path)
    df_name = "$(d_name)-results-$(today()).csv"

    println("======================= Domain: $d_name =======================")
    for (p_name::String, p_path::String) in instances[d_name]
        problem = load_problem(p_path)
        psize = length(PDDL.get_objects(problem))

        println("- Starting Problem: $p_name, with Size: $psize")
        
        # Initialize state and specification
        state = initstate(domain, problem)
        spec = Specification(problem)
        # Compile domain
        cdomain, cstate = compiled(domain, state)
        
        # Repeat for both original and compiled
        for dom in (domain, cdomain), planner_name in planners
            # Indicate if we do Compiled or Interpreted
            dom isa CompiledDomain ? println("----- Compiled ($planner_name): -----") : println("----- Interpreted ($planner_name): -----")
            state = initstate(dom, problem)
            #Create LM graph
            lm_graph::LandmarkGraph, gen_data::SymbolicPlanners.LandmarkGenerationData = compute_relaxed_landmark_graph(dom, state, spec)
            approximate_reasonable_orders(lm_graph, gen_data)
            size_landmarks = length(lm_graph.nodes)
            
            # Create appropriate planner based on planner_name
            planner = nothing
            if planner_name == "GoalCount"
                planner = AStarPlanner(GoalCountHeuristic(), max_time=TIMEOUT, save_search=true)
            elseif planner_name == "HAdd"
                planner = AStarPlanner(HAdd(), max_time=TIMEOUT, save_search=true)
            elseif planner_name == "LM_Count"
                planner = AStarPlanner(LMCount(deepcopy(lm_graph), gen_data.planning_graph), max_time=TIMEOUT, save_search=true)
            elseif planner_name == "LM_Local-HAdd"
                deep_lm_graph = deepcopy(lm_graph)
                landmark_graph_remove_initial_state(deep_lm_graph, gen_data.initial_state)
                landmark_graph_remove_cycles_fast(deep_lm_graph)
                size_landmarks = length(deep_lm_graph.nodes)
                planner = LMLocalPlanner(deepcopy(deep_lm_graph), gen_data.planning_graph, AStarPlanner(HAdd(), save_search=true), TIMEOUT)
            elseif planner_name == "LM_Local_Smart-HAdd"
                deep_lm_graph = deepcopy(lm_graph)
                landmark_graph_remove_initial_state(deep_lm_graph, gen_data.initial_state)
                landmark_graph_remove_cycles_fast(deep_lm_graph)
                size_landmarks = length(deep_lm_graph.nodes)
                planner = LMLocalSmartPlanner(deepcopy(deep_lm_graph), gen_data, AStarPlanner(HAdd(), save_search=true), TIMEOUT)
            end

            nruns = dom isa CompiledDomain ? NRUNS + 1 : NRUNS
            timed_out = false
            # Run and time planner
            for i in 1:nruns
                if timed_out continue end
                println("- Run: $i - LMGraph Size: $(length(lm_graph.nodes))")
                stats = @timed begin
                    sol = planner(dom, state, spec)
                end
                timed_out = sol.status == :max_time
                println("time: ", timed_out ? -1 : stats.time, " seconds")
                row = (
                    domain = d_name,
                    problem = p_name,
                    problem_size = psize,
                    compiled = (dom isa CompiledDomain),
                    planner = planner_name,
                    run = i,
                    time = timed_out ? -1.0 : stats.time,
                    bytes = stats.bytes,
                    n_steps = timed_out ? -1 : length(sol.plan),
                    n_eval = length(sol.search_tree),
                    n_expand = sol.expanded,
                    n_landmarks = size_landmarks,
                    correct_goal = is_goal(spec, dom, sol.trajectory[end])
                )
                push!(df, row)
                GC.gc()
            end
        end
        println()
    end
    CSV.write(df_name, df)
end