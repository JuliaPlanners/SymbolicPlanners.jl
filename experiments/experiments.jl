## Benchmarking experiments for planners on propositional domains ##
using PDDL, SymbolicPlanners, Test, PlanningDomains
using DataFrames, CSV, Dates, Statistics


planners = ["GoalCount", "HAdd", "LM_Count", "LM_Local-HAdd"]

TIMEOUT = 180.0
NRUNS = 3

# Store results in data frame
df = DataFrame(domain=String[], problem=String[], problem_size=Int[],
               compiled=Bool[], planner=String[], run=Int[],
               n_steps=Int[], time=Float64[], bytes=Int[],
               n_eval=Int[], n_expand=Int[], n_landmarks=Int[])

#JuliaPlannerRepo domains
for d_name in list_domains(JuliaPlannersRepo)
    domain = load_domain(JuliaPlannersRepo, d_name)
    df_name = "Julia-$(d_name)-results-$(today()).csv"

    println("======================= Julia Domain: $d_name =======================")
    for p_name in list_problems(JuliaPlannersRepo, d_name)
        problem = load_problem(JuliaPlannersRepo, d_name, p_name)
        psize = length(PDDL.get_objects(problem))

        println("- Starting Problem: $p_name, with Size: $psize")
        
        # Initialize state and specification
        state = initstate(domain, problem)
        spec = Specification(problem)
        # Compile domain
        cdomain, cstate = compiled(domain, state)
        #Create LM graph
        lm_graph::LandmarkGraph, p_graph::SymbolicPlanners.PlanningGraph = compute_landmark_graph(domain, state, spec)
        size_landmarks = length(lm_graph.nodes)

        # Repeat for both original and compiled
        for dom in (domain, cdomain), planner_name in planners
            # Indicate if we do Compiled or Interpreted
            dom isa CompiledDomain ? println("Compiled ($planner_name):") : println("Interpreted ($planner_name):")
            state = initstate(dom, problem)

            # Create appropriate planner based on planner_name
            planner = nothing
            if planner_name == "GoalCount"
                planner = AStarPlanner(GoalCountHeuristic(), max_time=TIMEOUT, save_search=true)
            elseif planner_name == "HAdd"
                planner = AStarPlanner(HAdd(), max_time=TIMEOUT, save_search=true)
            elseif planner_name == "LM_Count"
                planner = AStarPlanner(LMCount(lm_graph, p_graph), max_time=TIMEOUT, save_search=true)
            elseif planner_name == "LM_Local-HAdd"
                planner = LMLocalPlanner(lm_graph, gen_data.planning_graph, AStarPlanner(HAdd(), save_search=true))
            end

            nruns = dom isa CompiledDomain ? NRUNS + 1 : NRUNS
            timed_out = false
            # Run and time planner
            for i in 1:nruns
                if timed_out continue end
                println("- Run: $i ")
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
                    n_landmarks = size_landmarks
                )
                push!(df, row)
                GC.gc()
            end
        end
        println()
    end
    CSV.write(df_path, df)
end

#IPC 2014 domains
for d_name in list_domains(IPCInstancesRepo, "ipc-2014")
    domain = load_domain(IPCInstancesRepo, "ipc-2014", d_name)
    df_name = "IPC-2014-$(d_name)-results-$(today()).csv"

    println("======================= IPC Domain: $d_name =======================")
    for p_name in list_problems(IPCInstancesRepo, "ipc-2014", d_name)
        problem = load_problem(IPCInstancesRepo, "ipc-2014", d_name, p_name)
        psize = length(PDDL.get_objects(problem))

        println("- Starting Problem: $p_name, with Size: $psize")
        
        # Initialize state and specification
        state = initstate(domain, problem)
        spec = Specification(problem)
        # Compile domain
        cdomain, cstate = compiled(domain, state)
        #Create LM graph
        lm_graph::LandmarkGraph, p_graph::SymbolicPlanners.PlanningGraph = compute_landmark_graph(domain, state, spec)
        size_landmarks = length(lm_graph.nodes)

        # Repeat for both original and compiled
        for dom in (domain, cdomain), planner_name in planners
            # Indicate if we do Compiled or Interpreted
            dom isa CompiledDomain ? println("Compiled ($planner_name):") : println("Interpreted ($planner_name):")
            state = initstate(dom, problem)

            # Create appropriate planner based on planner_name
            planner = nothing
            if planner_name == "GoalCount"
                planner = AStarPlanner(GoalCountHeuristic(), max_time=TIMEOUT, save_search=true)
            elseif planner_name == "HAdd"
                planner = AStarPlanner(HAdd(), max_time=TIMEOUT, save_search=true)
            elseif planner_name == "LM_Count"
                planner = AStarPlanner(LMCount(lm_graph, p_graph), max_time=TIMEOUT, save_search=true)
            elseif planner_name == "LM_Local-HAdd"
                planner = LMLocalPlanner(lm_graph, gen_data.planning_graph, AStarPlanner(HAdd(), save_search=true), max_time=TIMEOUT)
            end

            nruns = dom isa CompiledDomain ? NRUNS + 1 : NRUNS
            timed_out = false
            # Run and time planner
            for i in 1:nruns
                if timed_out continue end
                println("- Run: $i ")
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
                    n_landmarks = size_landmarks
                )
                push!(df, row)
                GC.gc()
            end
        end
        println()
    end
    CSV.write(df_path, df)
end