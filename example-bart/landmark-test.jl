# Run using 'julia --project=. example-bart/landmark-test.jl'
# Can also be ran using 'julia ./example-bart/landmark-test.jl' after adding all the bellow packages in the julia pkg manager
using PDDL, SymbolicPlanners, Test, PlanningDomains

println("Started")
# Load Blocksworld domain and single problem
domain = load_domain(:blocksworld)
problem = load_problem(:blocksworld, "problem-3")

# Initialize state
state = initstate(domain, problem)
spec = Specification(problem)

#Create LM graph, Here is where you could add different Landmark Generation methods
lm_graph::LandmarkGraph, gen_data::SymbolicPlanners.LandmarkGenerationData = compute_relaxed_landmark_graph(domain, state, spec)
approximate_reasonable_orders(lm_graph, gen_data)
landmark_graph_remove_initial_state(lm_graph, gen_data.initial_state)
landmark_graph_remove_cycles_complete(lm_graph)


# Add our planner here, Chose one of the bellow to solve a problem.
planner = LMLocalPlanner(lm_graph, gen_data.planning_graph, AStarPlanner(HAdd(), save_search=true), 180.0)
# planner = AStarPlanner(LMCount(lm_graph, gen_data.planning_graph), max_time=180.0, save_search=true)
# planner = LMLocalSmartPlanner(lm_graph, gen_data, AStarPlanner(HAdd(), save_search=true), 180.0)

## Run Planner ##

println("Verifying interpreted")
stats = @timed begin
    sol = planner(domain, state, spec)
end

@test is_goal(spec, domain, sol.trajectory[end])
println("Interpreted Finished in ", stats.time, " seconds")

cdomain, cstate = compiled(domain, state)

println("Verifying compiled")
stats = @timed begin
    sol = planner(cdomain, cstate, spec)
end

@test is_goal(spec, cdomain, sol.trajectory[end])
println("Compiled Finished ", stats.time, " seconds")
