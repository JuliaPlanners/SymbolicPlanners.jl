@testset "Heuristics" begin

@testset "Interface" begin

goal_count = GoalCountHeuristic()
hval1 = goal_count(doors_keys_gems, dkg_problem)
hval2 = goal_count(doors_keys_gems, dkg_state, dkg_problem.goal)
hval3 = goal_count(doors_keys_gems, dkg_state, dkg_spec)
@test hval1 == hval2 == hval3

h_add = HAdd()
SymbolicPlanners.ensure_precomputed!(h_add, blocksworld, bw_state, bw_spec)
hval1 = h_add(blocksworld, bw_problem; precompute=false)
hval2 = h_add(blocksworld, bw_state, bw_problem.goal; precompute=false)
hval3 = h_add(blocksworld, bw_state, bw_spec; precompute=false)
@test hval1 == hval2 == hval3

end

@testset "Goal Count Heuristic" begin

goal_count = GoalCountHeuristic()
@test goal_count(gridworld, gw_state, gw_problem.goal) == 1
@test goal_count(doors_keys_gems, dkg_state, dkg_problem.goal) == 1
@test goal_count(blocksworld, bw_state, bw_problem.goal) == 2

gw_act_spec = ActionGoal(pddl"(up)")
@test goal_count(gridworld, gw_state, gw_act_spec) == 2
dkg_act_spec = ActionGoal(pddl"(unlock ?k ?d)")
@test goal_count(doors_keys_gems, dkg_state, dkg_act_spec) == 3
bw_act_spec = ActionGoal(pddl"(stack a ?x)", pddl"(on ?x c)")
@test goal_count(blocksworld, bw_state, bw_act_spec) == 2

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

@testset "PolicyValueHeuristic" begin

gw_state_2 = copy(gw_state)
gw_state_2[pddl"(ypos)"] = 3    

manhattan = precompute!(ManhattanHeuristic(@pddl("xpos", "ypos")),
                        gridworld, gw_state, gw_spec)
policy = HeuristicVPolicy(manhattan, gridworld, gw_spec)
heuristic = PolicyValueHeuristic(policy)

@test heuristic(gridworld, gw_state, gw_spec) == 2
@test heuristic(gridworld, gw_state_2, gw_spec) == 4

end

@testset "GoalDependentPolicyHeuristic" begin

gw_spec_2 = Specification(@pddl("(== xpos 1)", "(== ypos 1)"))
gw_state_2 = copy(gw_state)
gw_state_2[pddl"(ypos)"] = 3    

manhattan_1 = precompute!(ManhattanHeuristic(@pddl("xpos", "ypos")),
                          gridworld, gw_state, gw_spec)
manhattan_2 = precompute!(ManhattanHeuristic(@pddl("xpos", "ypos")),
                          gridworld, gw_state, gw_spec_2)
policies = Dict(
    gw_spec => HeuristicVPolicy(manhattan_1, gridworld, gw_spec),
    gw_spec_2 => HeuristicVPolicy(manhattan_2, gridworld, gw_spec_2)
)
heuristic = GoalDependentPolicyHeuristic(policies)

@test heuristic(gridworld, gw_state, gw_spec) == 2
@test heuristic(gridworld, gw_state, gw_spec_2) == 0
@test heuristic(gridworld, gw_state_2, gw_spec) == 4
@test heuristic(gridworld, gw_state_2, gw_spec_2) == 2

function default(domain, state, spec)
    manhattan = precompute!(ManhattanHeuristic(@pddl("xpos", "ypos")),
                            domain, state, spec)
    return HeuristicVPolicy(manhattan, domain, spec)
end

policies = Dict{MinStepsGoal, HeuristicVPolicy}()
heuristic = GoalDependentPolicyHeuristic(policies, default)

@test heuristic(gridworld, gw_state, gw_spec) == 2
@test heuristic(gridworld, gw_state, gw_spec_2) == 0
@test heuristic(gridworld, gw_state_2, gw_spec) == 4
@test heuristic(gridworld, gw_state_2, gw_spec_2) == 2

end 

@testset "HSP Heuristics" begin

# Test HSP heuristics (without axioms)
h_add = HAdd()
h_max = HMax()
@test h_add(blocksworld, bw_state, bw_problem.goal) == 4
@test h_max(blocksworld, bw_state, bw_problem.goal) == 2

# Test dynamic goal updating (without axioms)
precompute!(h_add, blocksworld, bw_state)
precompute!(h_max, blocksworld, bw_state)
@test compute(h_add, blocksworld, bw_state, bw_problem.goal) == 4
@test compute(h_max, blocksworld, bw_state, bw_problem.goal) == 2

# Test HSP heuristics with axioms
h_add = HAdd()
h_max = HMax()
@test h_add(bw_axioms, ba_state, ba_problem.goal) == 4
@test h_max(bw_axioms, ba_state, ba_problem.goal) == 2

# Test dynamic goal updating with axioms
precompute!(h_add, bw_axioms, ba_state)
precompute!(h_max, bw_axioms, ba_state)
@test compute(h_add, bw_axioms, ba_state, ba_problem.goal) == 4
@test compute(h_max, bw_axioms, ba_state, ba_problem.goal) == 2

# Test HSP heuristics with action goals
h_add = HAdd()
h_max = HMax()
bw_act_spec = ActionGoal(pddl"(pick-up a)")
@test h_add(blocksworld, bw_state, bw_act_spec) == 0
@test h_max(blocksworld, bw_state, bw_act_spec) == 0
bw_act_spec = ActionGoal(pddl"(stack a c)")
@test h_add(blocksworld, bw_state, bw_act_spec) == 1
@test h_max(blocksworld, bw_state, bw_act_spec) == 1
bw_act_spec = ActionGoal(pddl"(stack a ?x)", pddl"(on ?x c)")
@test h_add(blocksworld, bw_state, bw_act_spec) == 3
@test h_max(blocksworld, bw_state, bw_act_spec) == 2

# Test HSP heuristics with action costs
h_add = HAdd()
h_max = HMax()
bw_actions = collect(keys(PDDL.get_actions(blocksworld)))
bw_act_costs = ones(length(bw_actions)) .* 2
bw_act_spec = MinActionCosts([bw_problem.goal], bw_actions, bw_act_costs)
@test h_add(blocksworld, bw_state, bw_act_spec) == 8
@test h_max(blocksworld, bw_state, bw_act_spec) == 4

zt_infer_spec = zt_infer_spec = MinActionCosts(zeno_travel, zt_problem)
@test h_add(zeno_travel, zt_state, zt_infer_spec) == 44588
@test h_max(zeno_travel, zt_state, zt_infer_spec) == 15512

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
@test 2 <= ff(blocksworld, bw_state, bw_problem.goal) <= 4
@test 2 <= ff(bw_axioms, ba_state, ba_problem.goal) <= 4

# Test dynamic goal updating
precompute!(ff, blocksworld, bw_state)
@test 2 <= compute(ff, blocksworld, bw_state, bw_problem.goal) <= 4
precompute!(ff, bw_axioms, ba_state)
@test 2 <= compute(ff, bw_axioms, ba_state, ba_problem.goal) <= 4

# Test FF heuristic with action goals
ff = FFHeuristic()
bw_act_spec = ActionGoal(pddl"(pick-up a)")
@test ff(blocksworld, bw_state, bw_act_spec) == 0
bw_act_spec = ActionGoal(pddl"(stack a c)")
@test ff(blocksworld, bw_state, bw_act_spec) == 1
bw_act_spec = ActionGoal(pddl"(stack a ?x)", pddl"(on ?x c)")
@test ff(blocksworld, bw_state, bw_act_spec) == 3

# Test FF heuristic with action costs
ff = FFHeuristic()
bw_actions = collect(keys(PDDL.get_actions(blocksworld)))
bw_act_costs = ones(length(bw_actions)) .* 2
bw_act_spec = MinActionCosts([bw_problem.goal], bw_actions, bw_act_costs)
@test 4 <= ff(blocksworld, bw_state, bw_act_spec) <= 8

zt_infer_spec = zt_infer_spec = MinActionCosts(zeno_travel, zt_problem)
@test 15512 <= ff(zeno_travel, zt_state, zt_infer_spec) <= 44592

end

@testset "LM-Cut Heuristic" begin

lmcut = LMCut()
@test 4 >= lmcut(blocksworld, bw_state, bw_problem.goal) >= 2
@test 7 >= lmcut(wgc_domain, wgc_state, wgc_problem.goal) >= 1
@test 4 >= lmcut(bw_axioms, ba_state, ba_problem.goal) >= 2

# Test dynamic goal updating
precompute!(lmcut, blocksworld, bw_state)
@test 4 >= compute(lmcut, blocksworld, bw_state, bw_problem.goal) >= 2
precompute!(lmcut, wgc_domain, wgc_state)
@test 7 >= compute(lmcut, wgc_domain, wgc_state, wgc_problem.goal) >= 1
precompute!(lmcut, bw_axioms, ba_state)
@test 4 >= compute(lmcut, bw_axioms, ba_state, ba_problem.goal) >= 2

# Test that LM-Cut always dominates HMax
hmax = HMax()
@test lmcut(blocksworld, bw_state, bw_problem.goal) >=
    hmax(blocksworld, bw_state, bw_problem.goal)
@test lmcut(wgc_domain, wgc_state, wgc_problem.goal) >=
    hmax(wgc_domain, wgc_state, wgc_problem.goal)
@test lmcut(bw_axioms, ba_state, ba_problem.goal) >=
    hmax(bw_axioms, ba_state, ba_problem.goal)

# Test LMCut heuristic with action costs
lmcut = LMCut()
bw_actions = collect(keys(PDDL.get_actions(blocksworld)))
bw_act_costs = ones(length(bw_actions)) .* 2
bw_act_spec = MinActionCosts([bw_problem.goal], bw_actions, bw_act_costs)
@test 4 <= lmcut(blocksworld, bw_state, bw_act_spec) <= 8

zt_infer_spec = MinActionCosts(zeno_travel, zt_problem)
@test 15512 <= lmcut(zeno_travel, zt_state, zt_infer_spec) <= 44592

end

@testset "Reachability Heuristics" begin

reachability = ReachabilityHeuristic()
@test reachability(blocksworld, bw_state, bw_problem.goal) == 2
@test reachability(zeno_travel, zt_state, MinStepsGoal(zt_problem)) == 3
@test reachability(zeno_travel, zt_state, MinMetricGoal(zt_problem)) == 12

end

@testset "Precomputed Heuristic" begin

h = GoalCountHeuristic()
hval = h(doors_keys_gems, dkg_state, dkg_problem.goal)
h = precomputed(h, doors_keys_gems, dkg_state, dkg_problem.goal)
@test h(doors_keys_gems, dkg_state, dkg_problem.goal) == hval

h = FFHeuristic()
hval = h(blocksworld, bw_state, bw_problem.goal)
h = precomputed(FFHeuristic(), blocksworld, bw_state, bw_problem.goal)
@test h(blocksworld, bw_state, bw_problem.goal) == hval

end

@testset "Memoized Heuristic" begin

h = memoized(GoalCountHeuristic())
hval = h(doors_keys_gems, dkg_state, dkg_problem.goal)
@test h.heuristic(doors_keys_gems, dkg_state, dkg_problem.goal) == hval
@test h(doors_keys_gems, dkg_state, dkg_problem.goal) == hval

h = memoized(precomputed(FFHeuristic(), blocksworld, bw_state))
hval = h(blocksworld, bw_state, bw_problem.goal)
@test h.heuristic(blocksworld, bw_state, bw_problem.goal) == hval
@test h(blocksworld, bw_state, bw_problem.goal) == hval

end

end
