# Test that planners correctly solve simple problems

@testset "Planners" begin

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

end

@testset "Backward Planner" begin

planner = BackwardPlanner(heuristic=HAddR())
sol = planner(blocksworld, bw_state, bw_spec)
spec = SymbolicPlanners.BackwardSearchGoal(bw_spec, bw_state)
@test is_goal(spec, blocksworld, sol.trajectory[1])
@test collect(sol) == @pddl("(pick-up a)", "(stack a b)",
                            "(pick-up c)", "(stack c a)")

end

@testset "BiDirectional Planner" begin
  planner = BidirectionalPlanner(HAdd(), HAddR(); save_search = true)
  sol = planner(blocksworld, bw_state, bw_spec)
  spec = SymbolicPlanners.BackwardSearchGoal(bw_spec, bw_state)
  @test is_goal(spec, blocksworld, sol.trajectory[1])
  @test is_goal(bw_spec, blocksworld, sol.trajectory[end])
  @test collect(sol) == @pddl("(pick-up a)", "(stack a b)",
                              "(pick-up c)", "(stack c a)")
  @test length(sol.f_trajectory) <  length(sol.trajectory)
  @test length(sol.b_trajectory) <  length(sol.trajectory)
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

heuristic = GoalCountHeuristic()
planner = RTDP(heuristic=heuristic, rollout_noise=1.0, n_rollouts=10)
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

end

end
