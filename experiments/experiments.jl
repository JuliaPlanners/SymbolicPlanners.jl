## Benchmarking experiments for planners on propositional domains ##
using PDDL, SymbolicPlanners
using DataFrames, CSV, Dates, Statistics

println("Started")
println()

planners = ["FF", "LM_Count", "LM_Local-FF"]
benchmark_file = "ordered-landmarks-benchmark.txt"

TIMEOUT = 180.0
MAX_MEMORY = 7000000000.0
NRUNS = 4

df = DataFrame(domain=String[], problem=String[], problem_size=Int[], planner=String[], run=Int[], n_steps=Int[],
                time=Float64[], bytes=Int[], n_eval=Int[], n_expand=Int[], n_landmarks=Int[])

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

# Run benchmark
for (d_name::String, d_path::String) in domains
    domain = load_domain(d_path)
    df_name = replace("Julia-$(d_name)-results-$(now()).csv", ':' => '_')

    println("======================= Domain: $d_name =======================")
    for (p_name::String, p_path::String) in instances[d_name]
        problem = load_problem(p_path)
        psize = length(PDDL.get_objects(problem))

        println("- Starting Problem: $p_name, with Size: $psize")
        
        state = initstate(domain, problem)
        spec = Specification(problem)
        cdomain, cstate = compiled(domain, state)
        
        # Create LM graph
        rg = compute_relaxed_landmark_graph(cdomain, cstate, spec, TIMEOUT)
        lm_graph = nothing
        p_graph = nothing
        if !isnothing(rg)
            approximate_reasonable_orders(rg.first, rg.second)
            lm_graph = rg.first
            p_graph = rg.second.planning_graph
        end

        # Solve using all planners
        for planner_name in planners
            println(" - $planner_name:")

            # Run and time planner
            timed_out = false
            for i in 1:NRUNS
                if timed_out continue end

                planner = nothing
                lm_num = -1
                if planner_name == "FF"
                    planner = AStarPlanner(FFHeuristic(), max_time=TIMEOUT, max_mem=MAX_MEMORY, save_search=true)
                elseif planner_name == "LM_Count" && !isnothing(lm_graph)
                    lm_num = length(lm_graph.nodes)
                    planner = AStarPlanner(LMCount(lm_graph, p_graph), max_time=TIMEOUT, max_mem=MAX_MEMORY, save_search=true)
                elseif planner_name == "LM_Local-FF" && !isnothing(lm_graph)
                    graphcopy = deepcopy(lm_graph)
                    landmark_graph_remove_cycles_complete(graphcopy)
                    lm_num = length(graphcopy.nodes)
                    planner = LMLocalPlanner(graphcopy, p_graph, AStarPlanner(FFHeuristic(), max_time=TIMEOUT, max_mem=MAX_MEMORY, save_search=true), TIMEOUT, MAX_MEMORY)
                else
                    continue
                end
                println("  - Run: $i - LMGraph Size: $lm_num")

                # Run planner if valid
                time = -1
                bytes = -1
                n_steps = -1
                n_eval = -1
                n_expand = -1
                if !isnothing(planner)
                    stats = @timed begin
                        sol = planner(cdomain, cstate, spec)
                    end
                    if !is_goal(spec, domain, GenericState(sol.trajectory[end]))
                        println("no valid solution", )
                    end
                    timed_out = sol.status == :max_time && i != 1
                    n_steps = sol.status == :max_time ? -1 : length(sol.plan)
                    time = sol.status == :max_time ? -1.0 : stats.time
                    bytes = stats.bytes
                    n_eval = length(sol.search_tree)
                    n_expand = sol.expanded
                end

                # Save result
                println("  time: ", time, " seconds, steps: ", n_steps)
                row = (
                    domain = d_name,
                    problem = p_name,
                    problem_size = psize,
                    planner = planner_name,
                    run = i,
                    time = time,
                    bytes = bytes,
                    n_steps = n_steps,
                    n_eval = n_eval,
                    n_expand = n_expand,
                    n_landmarks = lm_num
                )
                push!(df, row)
                GC.gc()
            end
            println()
        end
    end
    CSV.write(df_name, df)
end
