@testset "Heuristics" begin

@testset "Goal Count Heuristic" begin

goal_count = GoalCountHeuristic()
@test goal_count(gridworld, gw_state, gw_problem.goal) == 1
@test goal_count(doors_keys_gems, dkg_state, dkg_problem.goal) == 1
@test goal_count(blocksworld, bw_state, bw_problem.goal) == 2

end

@testset "Metric Heuristic" begin

gw_state_2 = copy(gw_state)
gw_state_2[pddl"(ypos)"] = 3

manhattan = ManhattanHeuristic(@pddl("xpos", "ypos"))
@test manhattan(gridworld, gw_state, gw_problem.goal) == 2
@test manhattan(gridworld, gw_state_2, gw_problem.goal) == 4

euclidean = EuclideanHeuristic(@pddl("xpos", "ypos"))
@test euclidean(gridworld, gw_state, gw_problem.goal) == 2
@test euclidean(gridworld, gw_state_2, gw_problem.goal) ≈ sqrt(8)

end

@testset "Planner Heuristic" begin

# Test planner heuristic without state or domain transform
planner = AStarPlanner(ManhattanHeuristic(@pddl("xpos", "ypos")))
heuristic = PlannerHeuristic(planner)
@test heuristic(gridworld, gw_state, gw_problem.goal) == 6

# Test planner heuristic with state transform
planner = AStarPlanner(GoalCountHeuristic())
remove_doors = s -> begin
    s = copy(s)
    for d in PDDL.get_objects(s, :door)
        s[Compound(:locked, Term[d])] = false
    end
    return s
end
heuristic = PlannerHeuristic(planner, s_transform=remove_doors)
@test heuristic(doors_keys_gems, dkg_state, dkg_problem.goal) == 7

end

@testset "HSP Heuristics" begin

h_add = HAdd()
h_max = HMax()
@test h_add(blocksworld, bw_state, bw_problem.goal) == 4
@test h_max(blocksworld, bw_state, bw_problem.goal) == 2

# Test dynamic goal updating
precompute!(h_add, blocksworld, bw_state)
precompute!(h_max, blocksworld, bw_state)
@test compute(h_add, blocksworld, bw_state, bw_problem.goal) == 4
@test compute(h_max, blocksworld, bw_state, bw_problem.goal) == 2

h_add = HAdd()
h_max = HMax()
@test h_add(bw_axioms, ba_state, ba_problem.goal) == 4
@test h_max(bw_axioms, ba_state, ba_problem.goal) == 2

# Test dynamic goal updating
precompute!(h_add, bw_axioms, ba_state)
precompute!(h_max, bw_axioms, ba_state)
@test compute(h_add, bw_axioms, ba_state, ba_problem.goal) == 4
@test compute(h_max, bw_axioms, ba_state, ba_problem.goal) == 2

end

@testset "HSPr Heuristics" begin

bw_init = initstate(blocksworld, bw_problem)
bw_goal = goalstate(blocksworld, bw_problem)
h_add_r = precomputed(HAddR(), blocksworld, bw_init, bw_problem.goal)
h_max_r = precomputed(HMaxR(), blocksworld, bw_init, bw_problem.goal)

@test h_add_r(blocksworld, bw_goal, bw_problem.goal) == 4
@test h_max_r(blocksworld, bw_goal, bw_problem.goal) == 2

end

@testset "FF Heuristic" begin

ff = FFHeuristic()
@test ff(blocksworld, bw_state, bw_problem.goal) <= 4
@test ff(bw_axioms, ba_state, ba_problem.goal) <= 4

# Test dynamic goal updating
precompute!(ff, blocksworld, bw_state)
@test compute(ff, blocksworld, bw_state, bw_problem.goal) <= 4
precompute!(ff, bw_axioms, ba_state)
@test compute(ff, bw_axioms, ba_state, ba_problem.goal) <= 4

end

@testset "Reachability Heuristics" begin

reachability = ReachabilityHeuristic()
@test reachability(blocksworld, bw_state, bw_problem.goal) == 2
@test reachability(zeno_travel, zt_state, MinStepsGoal(zt_problem)) == 3
@test reachability(zeno_travel, zt_state, MinMetricGoal(zt_problem)) == 12

end


end
