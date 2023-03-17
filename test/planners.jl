# Test that planners correctly solve simple problems

@testset "Planners" begin

@testset "Interface" begin

planner = AStarPlanner(GoalCountHeuristic())
sol1 = planner(doors_keys_gems, dkg_problem)
sol2 = planner(doors_keys_gems, dkg_state, dkg_problem.goal)
sol3 = planner(doors_keys_gems, dkg_state, dkg_spec)
@test sol1 == sol2 == sol3

planner = AStarPlanner(GoalCountHeuristic(), save_search=true)
sol1 = planner(doors_keys_gems, dkg_state, dkg_spec)
sol2 = refine!(sol1, planner, doors_keys_gems, dkg_state, dkg_spec)
sol3 = refine(sol1, planner, doors_keys_gems, dkg_state, dkg_problem.goal)
@test sol1 === sol2 == sol3

planner = AStarPlanner(HAdd())
sol1 = planner(blocksworld, bw_problem)
sol2 = planner(blocksworld, bw_state, bw_problem.goal)
sol3 = planner(blocksworld, bw_state, bw_spec)
@test sol1 == sol2 == sol3

planner = AStarPlanner(HAdd(), save_search=true)
sol1 = planner(blocksworld, bw_state, bw_spec)
sol2 = refine!(sol1, planner, blocksworld, bw_state, bw_spec)
sol3 = refine(sol1, planner, blocksworld, bw_state, bw_problem.goal)
@test sol1 === sol2 == sol3

end

@testset "Breadth-First Planner" begin

planner = BreadthFirstPlanner()
sol = planner(gridworld, gw_state, gw_spec)
@test is_goal(gw_spec, gridworld, sol.trajectory[end])
@test collect(sol) == @pddl("down", "down", "right", "right", "up", "up")

sol = planner(doors_keys_gems, dkg_state, dkg_spec)
@test is_goal(dkg_spec, doors_keys_gems, sol.trajectory[end])
@test collect(sol) == @pddl("(down)", "(pickup key1)", "(down)",
                            "(unlock key1 door1)", "(right)", "(right)",
                            "(up)", "(up)", "(pickup gem1)")

sol = planner(blocksworld, bw_state, bw_spec)
@test is_goal(bw_spec, blocksworld, sol.trajectory[end])
@test collect(sol) == @pddl("(pick-up a)", "(stack a b)",
                            "(pick-up c)", "(stack c a)")

sol = planner(wgc_domain, wgc_state, wgc_spec)
@test is_goal(wgc_spec, wgc_domain, sol.trajectory[end])
@test |(
    collect(sol) == @pddl("(ship-goat left right)", "(ship-self right left)",
                          "(ship-cabbage left right)", "(ship-goat right left)",
                          "(ship-wolf left right)", "(ship-self right left)",
                          "(ship-goat left right)",),
    collect(sol) == @pddl("(ship-goat left right)", "(ship-self right left)",
                          "(ship-wolf left right)", "(ship-goat right left)",
                          "(ship-cabbage left right)", "(ship-self right left)",
                          "(ship-goat left right)",))

# Test solution refinement
planner = BreadthFirstPlanner(max_nodes=2, save_search=true)
sol = planner(blocksworld, bw_state, bw_spec)
planner.max_nodes = typemax(Int)
refine!(sol, planner, blocksworld, bw_state, bw_spec)
@test is_goal(bw_spec, blocksworld, sol.trajectory[end])
@test collect(sol) == @pddl("(pick-up a)", "(stack a b)",
                            "(pick-up c)", "(stack c a)")

@test copy(planner) == planner

end

@testset "Uniform Cost Planner" begin

planner = UniformCostPlanner()
sol = planner(gridworld, gw_state, gw_spec)
@test is_goal(gw_spec, gridworld, sol.trajectory[end])
@test collect(sol) == @pddl("down", "down", "right", "right", "up", "up")

planner = UniformCostPlanner()
sol = planner(doors_keys_gems, dkg_state, dkg_spec)
@test is_goal(dkg_spec, doors_keys_gems, sol.trajectory[end])
@test collect(sol) == @pddl("(down)", "(pickup key1)", "(down)",
                            "(unlock key1 door1)", "(right)", "(right)",
                            "(up)", "(up)", "(pickup gem1)")

planner = UniformCostPlanner()
sol = planner(blocksworld, bw_state, bw_spec)
@test is_goal(bw_spec, blocksworld, sol.trajectory[end])
@test collect(sol) == @pddl("(pick-up a)", "(stack a b)",
                            "(pick-up c)", "(stack c a)")
planner = UniformCostPlanner()

sol = planner(wgc_domain, wgc_state, wgc_spec)
@test is_goal(wgc_spec, wgc_domain, sol.trajectory[end])
@test |(
    collect(sol) == @pddl("(ship-goat left right)", "(ship-self right left)",
                          "(ship-cabbage left right)", "(ship-goat right left)",
                          "(ship-wolf left right)", "(ship-self right left)",
                          "(ship-goat left right)",),
    collect(sol) == @pddl("(ship-goat left right)", "(ship-self right left)",
                          "(ship-wolf left right)", "(ship-goat right left)",
                          "(ship-cabbage left right)", "(ship-self right left)",
                          "(ship-goat left right)",))

@test copy(planner) == planner

end

@testset "Greedy Planner" begin

planner = GreedyPlanner(ManhattanHeuristic(@pddl("xpos", "ypos")))
sol = planner(gridworld, gw_state, gw_spec)
@test is_goal(gw_spec, gridworld, sol.trajectory[end])
@test collect(sol) == @pddl("down", "down", "right", "right", "up", "up")

planner = GreedyPlanner(GoalCountHeuristic())
sol = planner(doors_keys_gems, dkg_state, dkg_spec)
@test is_goal(dkg_spec, doors_keys_gems, sol.trajectory[end])
@test collect(sol) == @pddl("(down)", "(pickup key1)", "(down)",
                            "(unlock key1 door1)", "(right)", "(right)",
                            "(up)", "(up)", "(pickup gem1)")

planner = GreedyPlanner(HAdd())
sol = planner(blocksworld, bw_state, bw_spec)
@test is_goal(bw_spec, blocksworld, sol.trajectory[end])
@test collect(sol) == @pddl("(pick-up a)", "(stack a b)",
                            "(pick-up c)", "(stack c a)")

planner = GreedyPlanner(HAdd())
sol = planner(zeno_travel, zt_state, zt_spec)
@test is_goal(zt_spec, zeno_travel, sol.trajectory[end])

planner = GreedyPlanner(HAdd())
sol = planner(wgc_domain, wgc_state, wgc_spec)
@test is_goal(wgc_spec, wgc_domain, sol.trajectory[end])
@test |(
    collect(sol) == @pddl("(ship-goat left right)", "(ship-self right left)",
                          "(ship-cabbage left right)", "(ship-goat right left)",
                          "(ship-wolf left right)", "(ship-self right left)",
                          "(ship-goat left right)",),
    collect(sol) == @pddl("(ship-goat left right)", "(ship-self right left)",
                          "(ship-wolf left right)", "(ship-goat right left)",
                          "(ship-cabbage left right)", "(ship-self right left)",
                          "(ship-goat left right)",))

@test copy(planner) == planner

end

@testset "A* Planner" begin

planner = AStarPlanner(ManhattanHeuristic(@pddl("xpos", "ypos")))
sol = planner(gridworld, gw_state, gw_spec)
@test is_goal(gw_spec, gridworld, sol.trajectory[end])
@test collect(sol) == @pddl("down", "down", "right", "right", "up", "up")

planner = AStarPlanner(GoalCountHeuristic())
sol = planner(doors_keys_gems, dkg_state, dkg_spec)
@test is_goal(dkg_spec, doors_keys_gems, sol.trajectory[end])
@test collect(sol) == @pddl("(down)", "(pickup key1)", "(down)",
                            "(unlock key1 door1)", "(right)", "(right)",
                            "(up)", "(up)", "(pickup gem1)")

planner = AStarPlanner(HAdd())
sol = planner(blocksworld, bw_state, bw_spec)
@test is_goal(bw_spec, blocksworld, sol.trajectory[end])
@test collect(sol) == @pddl("(pick-up a)", "(stack a b)",
                            "(pick-up c)", "(stack c a)")

planner = AStarPlanner(HAdd())
sol = planner(zeno_travel, zt_state, zt_spec)
@test is_goal(zt_spec, zeno_travel, sol.trajectory[end])

planner = AStarPlanner(HAdd())
sol = planner(wgc_domain, wgc_state, wgc_spec)
@test is_goal(wgc_spec, wgc_domain, sol.trajectory[end])
@test |(
    collect(sol) == @pddl("(ship-goat left right)", "(ship-self right left)",
                          "(ship-cabbage left right)", "(ship-goat right left)",
                          "(ship-wolf left right)", "(ship-self right left)",
                          "(ship-goat left right)",),
    collect(sol) == @pddl("(ship-goat left right)", "(ship-self right left)",
                          "(ship-wolf left right)", "(ship-goat right left)",
                          "(ship-cabbage left right)", "(ship-self right left)",
                          "(ship-goat left right)",))

planner = AStarPlanner(HAdd(), max_nodes=2, save_search=true)
sol = planner(blocksworld, bw_state, bw_spec)
@test sol.status == :max_nodes
planner.max_nodes = typemax(Int)
refine!(sol, planner, blocksworld, bw_state, bw_spec)
@test sol.status == :success
@test is_goal(bw_spec, blocksworld, sol.trajectory[end])
@test collect(sol) == @pddl("(pick-up a)", "(stack a b)",
                            "(pick-up c)", "(stack c a)")

@test copy(planner) == planner

end

@testset "Probabilistic A* Planner" begin

Random.seed!(0)

planner = ProbAStarPlanner(ManhattanHeuristic(@pddl("xpos", "ypos")))
sol = planner(gridworld, gw_state, gw_spec)
@test is_goal(gw_spec, gridworld, sol.trajectory[end])
@test collect(sol) == @pddl("down", "down", "right", "right", "up", "up")

planner = ProbAStarPlanner(GoalCountHeuristic())
sol = planner(doors_keys_gems, dkg_state, dkg_spec)
@test is_goal(dkg_spec, doors_keys_gems, sol.trajectory[end])
@test collect(sol) == @pddl("(down)", "(pickup key1)", "(down)",
                            "(unlock key1 door1)", "(right)", "(right)",
                            "(up)", "(up)", "(pickup gem1)")

planner = ProbAStarPlanner(HAdd())
sol = planner(blocksworld, bw_state, bw_spec)
@test is_goal(bw_spec, blocksworld, sol.trajectory[end])
@test collect(sol) == @pddl("(pick-up a)", "(stack a b)",
                            "(pick-up c)", "(stack c a)")

# Test that search order differs with enough randomness
planner = ProbAStarPlanner(
    GoalCountHeuristic(), search_noise=2.0, 
    save_search=true, save_search_order=true
)
Random.seed!(1)
sol1 = planner(doors_keys_gems, dkg_state, dkg_spec)
Random.seed!(2)
sol2 = planner(doors_keys_gems, dkg_state, dkg_spec)
@test sol1.search_order != sol2.search_order

# Test solution refinement
planner = ProbAStarPlanner(HAdd(), max_nodes=2, save_search=true)
sol = planner(blocksworld, bw_state, bw_spec)
@test sol.status == :max_nodes
planner.max_nodes = typemax(Int)
refine!(sol, planner, blocksworld, bw_state, bw_spec)
@test sol.status == :success
@test is_goal(bw_spec, blocksworld, sol.trajectory[end])
@test collect(sol) == @pddl("(pick-up a)", "(stack a b)",
                            "(pick-up c)", "(stack c a)")

@test copy(planner) == planner

end

@testset "Backward Planner" begin

planner = BackwardPlanner(heuristic=HAddR())
sol = planner(blocksworld, bw_state, bw_spec)
spec = SymbolicPlanners.BackwardSearchGoal(bw_spec, bw_state)
@test is_goal(spec, blocksworld, sol.trajectory[1])
@test collect(sol) == @pddl("(pick-up a)", "(stack a b)",
                            "(pick-up c)", "(stack c a)")

# Test solution refinement
planner = BackwardPlanner(heuristic=HAddR(), max_nodes=2, save_search=true)
sol = planner(blocksworld, bw_state, bw_spec)
@test sol.status == :max_nodes
planner.max_nodes = typemax(Int)
refine!(sol, planner, blocksworld, bw_state, bw_spec)
@test sol.status == :success
@test is_goal(spec, blocksworld, sol.trajectory[1])
@test collect(sol) == @pddl("(pick-up a)", "(stack a b)",
                            "(pick-up c)", "(stack c a)")

@test copy(planner) == planner

end

@testset "Probabilistic Backward Planner" begin

Random.seed!(0)

planner = ProbBackwardPlanner(heuristic=HAddR(), search_noise=0.1)
sol = planner(blocksworld, bw_state, bw_spec)
spec = SymbolicPlanners.BackwardSearchGoal(bw_spec, bw_state)
@test is_goal(spec, blocksworld, sol.trajectory[1])
@test collect(sol) == @pddl("(pick-up a)", "(stack a b)",
                            "(pick-up c)", "(stack c a)")

# Test that search order differs with enough randomness
planner = ProbBackwardPlanner(heuristic=HAddR(), search_noise=0.1, 
                              save_search=true, save_search_order=true)
Random.seed!(1)
sol1 = planner(blocksworld, bw_state, bw_spec)
Random.seed!(2)
sol2 = planner(blocksworld, bw_state, bw_spec)
@test sol1.search_order != sol2.search_order

# Test solution refinement
planner = ProbBackwardPlanner(heuristic=HAddR(), max_nodes=2,
                              search_noise=0.1, save_search=true)
sol = planner(blocksworld, bw_state, bw_spec)
@test sol.status == :max_nodes
planner.max_nodes = typemax(Int)
refine!(sol, planner, blocksworld, bw_state, bw_spec)
@test sol.status == :success
@test is_goal(spec, blocksworld, sol.trajectory[1])
@test collect(sol) == @pddl("(pick-up a)", "(stack a b)",
                            "(pick-up c)", "(stack c a)")

@test copy(planner) == planner

end

@testset "Bidirectional Planner" begin

planner = BidirectionalPlanner(HAdd(), HAddR(), save_search=true)
sol = planner(blocksworld, bw_state, bw_spec)
spec = SymbolicPlanners.BackwardSearchGoal(bw_spec, bw_state)
@test is_goal(spec, blocksworld, sol.trajectory[1])
@test is_goal(bw_spec, blocksworld, sol.trajectory[end])
@test collect(sol) == @pddl("(pick-up a)", "(stack a b)",
                            "(pick-up c)", "(stack c a)")

@test isnothing(sol.f_trajectory) || length(sol.f_trajectory) <  length(sol.trajectory)
@test isnothing(sol.b_trajectory) || length(sol.b_trajectory) <  length(sol.trajectory)

# Test solution refinement
planner = BidirectionalPlanner(HAdd(), HAddR(), save_search=true, max_nodes=2)
sol = planner(blocksworld, bw_state, bw_spec)
@test sol.status == :max_nodes
planner.max_nodes = typemax(Int)
refine!(sol, planner, blocksworld, bw_state, bw_spec)
@test sol.status == :success
@test is_goal(spec, blocksworld, sol.trajectory[1])
@test collect(sol) == @pddl("(pick-up a)", "(stack a b)",
                            "(pick-up c)", "(stack c a)")

@test copy(planner) == planner

end

@testset "Real Time Dynamic Programming" begin

Random.seed!(0)
simulator = StateActionRecorder(100)

heuristic = memoized(ManhattanHeuristic(@pddl("xpos", "ypos")))
planner = RTDP(heuristic=heuristic, rollout_noise=1.0, n_rollouts=10)
sol = planner(gridworld, gw_state, gw_spec)
actions, trajectory = simulator(sol, gridworld, gw_state, gw_spec)
@test is_goal(gw_spec, gridworld, trajectory[end])
@test actions == @pddl("down", "down", "right", "right", "up", "up")

planner = RTDP(heuristic=GoalCountHeuristic(), rollout_noise=1.0, n_rollouts=10)
sol = planner(doors_keys_gems, dkg_state, dkg_spec)
actions, trajectory = simulator(sol, doors_keys_gems, dkg_state, dkg_spec)
@test is_goal(dkg_spec, doors_keys_gems, trajectory[end])
@test actions == @pddl("(down)", "(pickup key1)", "(down)",
                       "(unlock key1 door1)", "(right)", "(right)",
                       "(up)", "(up)", "(pickup gem1)")

planner = RTDP(heuristic=HAdd(), rollout_noise=1.0, n_rollouts=10)
sol = planner(blocksworld, bw_state, bw_spec)
actions, trajectory = simulator(sol, blocksworld, bw_state, bw_spec)
@test is_goal(bw_spec, blocksworld, trajectory[end])
@test actions == @pddl("(pick-up a)", "(stack a b)",
                       "(pick-up c)", "(stack c a)")

# Test solution refinement
planner = RTDP(heuristic=GoalCountHeuristic(), rollout_noise=1.0, n_rollouts=1)
sol = planner(doors_keys_gems, dkg_state, dkg_spec)
actions, trajectory = simulator(sol, doors_keys_gems, dkg_state, dkg_spec)
@test !is_goal(dkg_spec, doors_keys_gems, trajectory[end])
planner.n_rollouts = 9
refine!(sol, planner, doors_keys_gems, dkg_state, dkg_spec)
actions, trajectory = simulator(sol, doors_keys_gems, dkg_state, dkg_spec)
@test is_goal(dkg_spec, doors_keys_gems, trajectory[end])
@test actions == @pddl("(down)", "(pickup key1)", "(down)",
                       "(unlock key1 door1)", "(right)", "(right)",
                       "(up)", "(up)", "(pickup gem1)")

@test copy(planner) == planner

end

@testset "Real Time Heuristic Search" begin

Random.seed!(0)
simulator = StateActionRecorder(100)

heuristic = ManhattanHeuristic(@pddl("xpos", "ypos"))
planner = RTHS(heuristic, n_iters=1, max_nodes=20)
sol = planner(gridworld, gw_state, gw_spec)
actions, trajectory = simulator(sol, gridworld, gw_state, gw_spec)
@test is_goal(gw_spec, gridworld, trajectory[end])
@test actions == @pddl("down", "down", "right", "right", "up", "up")

planner = RTHS(GoalCountHeuristic(), n_iters=10, max_nodes=20)
sol = planner(doors_keys_gems, dkg_state, dkg_spec)
actions, trajectory = simulator(sol, doors_keys_gems, dkg_state, dkg_spec)
@test is_goal(dkg_spec, doors_keys_gems, trajectory[end])
@test actions == @pddl("(down)", "(pickup key1)", "(down)",
                       "(unlock key1 door1)", "(right)", "(right)",
                       "(up)", "(up)", "(pickup gem1)")

planner = RTHS(HMax(), n_iters=5, max_nodes=10)
sol = planner(blocksworld, bw_state, bw_spec)
actions, trajectory = simulator(sol, blocksworld, bw_state, bw_spec)
@test is_goal(bw_spec, blocksworld, trajectory[end])
@test actions == @pddl("(pick-up a)", "(stack a b)",
                       "(pick-up c)", "(stack c a)")

# Test solution refinement
planner = RTHS(heuristic=GoalCountHeuristic(), n_iters=1, max_nodes=20)
sol = planner(doors_keys_gems, dkg_state, dkg_spec)
actions, trajectory = simulator(sol, doors_keys_gems, dkg_state, dkg_spec)
@test !is_goal(dkg_spec, doors_keys_gems, trajectory[end])
planner.n_iters = 9
refine!(sol, planner, doors_keys_gems, dkg_state, dkg_spec)
actions, trajectory = simulator(sol, doors_keys_gems, dkg_state, dkg_spec)
@test is_goal(dkg_spec, doors_keys_gems, trajectory[end])
@test actions == @pddl("(down)", "(pickup key1)", "(down)",
                       "(unlock key1 door1)", "(right)", "(right)",
                       "(up)", "(up)", "(pickup gem1)")

@test copy(planner) == planner

end

@testset "Monte Carlo Tree Search" begin

Random.seed!(0)
simulator = StateActionRecorder(100)

planner = MCTS(n_rollouts=50)
sol = planner(gridworld, gw_state, GoalReward(gw_problem.goal, 100))
actions, trajectory = simulator(sol, gridworld, gw_state, gw_spec)
@test is_goal(gw_spec, gridworld, trajectory[end])
@test actions == @pddl("down", "down", "right", "right", "up", "up")

planner = MCTS(n_rollouts=50)
sol = planner(doors_keys_gems, dkg_state, GoalReward(dkg_problem.goal, 100))
actions, trajectory = simulator(sol, doors_keys_gems, dkg_state, dkg_spec)
@test is_goal(dkg_spec, doors_keys_gems, trajectory[end])
@test actions == @pddl("(down)", "(pickup key1)", "(down)",
                       "(unlock key1 door1)", "(right)", "(right)",
                       "(up)", "(up)", "(pickup gem1)")

planner = MCTS(n_rollouts=50)
sol = planner(blocksworld, bw_state, GoalReward(bw_problem.goal, 100))
actions, trajectory = simulator(sol, blocksworld, bw_state, bw_spec)
@test is_goal(bw_spec, blocksworld, trajectory[end])
@test actions == @pddl("(pick-up a)", "(stack a b)",
                       "(pick-up c)", "(stack c a)")

@test copy(planner) == planner

end

end
