using Revise, PDDL, SymbolicPlanners, DataStructures.Queue, Test

@testset "urbanovska lmcut tests" begin
    

function test_lmcut_urban(problem_num; goal_num=1, c=nothing)
    p = load_problem("./src/heuristics/problems_pddl/pddl_0$problem_num/p$goal_num.pddl");
    domain = load_domain("./src/heuristics/problems_pddl/pddl_0$problem_num/domain.pddl");
    spec = Specification(p)
    state = initstate(domain, p)
    
    if (hmax_wrapper(domain, state, spec) == typemax(Float32))
        return typemax(Float32)
    end

    #println("hi")
    O = groundactions(domain, state)
    F = alt_facts(O)
    output_val = 0

    #Modify the initial problem
    F_m = deepcopy(F)
    O_m = deepcopy(O)
    init_m = pddl"(is init )"
    goal_m = pddl"(is goal )"

    diff_init = PDDL.GenericDiff(collect(state.facts), Vector(), Dict())
    o_init = GroundAction(:o_init, pddl"(action o_init)", [pddl"(is init)"], diff_init)

    diff_goal = PDDL.GenericDiff([goal_m], Vector(), Dict())
    
    o_goal = GroundAction(:o_goal, goal_m, spec.terms, diff_goal)
    

    push!(O_m, o_init)
    push!(O_m, o_goal)

    push!(F_m, init_m)
    push!(F_m, goal_m)

    

    c[o_init.name] = 0
    c[o_goal.name] = 0


    i = 1
    hmax_val, delta = hmax_with_cost(F_m, O_m, [init_m], [goal_m], c, domain)
    while hmax_val != 0
        #Find supporters
        supporters = Dict()
        for o in O_m
            if length(o.preconds) == 1
                supporters[o.name] = o.preconds[1]
                continue
            end
            supporters[o.name] = reduce((x,y) -> delta[x] > delta[y] ? x : y, o.preconds)
        end

        #edges
        graph = []
        for o in O_m
            for add in o.effect.add
                push!(graph, (supporters[o.name], add, o.name))
            end
        end

        #N* partition
        n_star = []
        q = Queue{Compound}()
        enqueue!(q, goal_m)
        push!(n_star, goal_m)
        while !isempty(q)
            fact = dequeue!(q)
            for g in graph
                if g[2] == fact && c[g[3]] == 0
                    enqueue!(q,g[1])
                    push!(n_star, g[1])
                end
            end
        end

        #N0
        n_zero = []
        q = Queue{Compound}()
        enqueue!(q, init_m)
        push!(n_zero, init_m)
        while !isempty(q)
            fact = dequeue!(q)
            for g in graph
                if g[1] == fact && !(g[2] in n_star) && !(g[2] in n_zero)
                    enqueue!(q,g[2])
                    push!(n_zero, g[2])
                end
            end
        end

        #Nb
        Nb = []
        for f in F_m
            if !(f in n_star) && !(f in n_zero)
                push!(Nb, f)
            end
        end

        #find landmarks and the minimal landmarks
        mi = typemax(Float32)
        mi_o = nothing
        Lm = []
        for o in O_m
            for g in graph
                if o.name == g[3] && g[1] in n_zero && g[2] in n_star
                    push!(Lm, o.name)
                    if c[o.name] < mi
                        mi = c[o.name]
                        mi_o = o.name
                    end
                end
            end
        end
        #update heurtisti value and cost function
        output_val += mi
        for o in Lm
            c[o] -= mi
        end
        hmax_val, delta = hmax_with_cost(F_m, O_m, [init_m], [goal_m], c, domain)
    end
    return output_val
end
function _get_action_cost_dict(O, spec)
    c = Dict{Symbol, Float32}()
    if has_action_cost(spec)
        for o in O
            #println(o)
            c[o.name] = get_action_cost(spec, o)
        end
    else
        for o in O
            #println(o)
            c[o.name] = 1
        end    
    end
    return c
end
function hmax_with_cost(F, Os, state, goal, c, domain)
    delta = Dict()
    U = Dict()
    C = Vector{Term}()
    if isa(goal, Compound)
        goal = [goal]
    end

    for f in F 
        delta[f] = typemax(Float32)
    end
    for fact in state
        delta[fact] = 0
    end
    for o in Os
        U[o] = length(o.preconds)
        if length(o.preconds) > 0           
            continue
        else
            for fact in o.effect.add
               delta[fact] = min(get!(delta, fact, Float32.maximum), c[o.name]) 

            end
        end
    end

    #A "fake" (fact only) state is created here solely to use the satisfy function
    currentState = GenericState(Set(), Set(C), Dict())
    while !satisfy(domain, currentState, goal)
        #https://stackoverflow.com/questions/33111909/julia-how-to-find-the-key-for-the-min-max-value-of-a-dict        
        k = reduce((x,y) -> delta[x] <= delta[y] ? x : y, setdiff(F, C))
    
        push!(C, k)

        for o in Os
            if k in o.preconds
                #println(k)
                U[o] -= 1
                if U[o] == 0
                    for add in o.effect.add
                        g = get!(delta, add, typemax(Float32))
                        delta[add] = min(g, c[o.name] + delta[k])
                    end
                end
            end
        end
        currentState = GenericState(Set(), Set(C), Dict())
    end
    #println("end")
    #@show delta
    return maximum([delta[f] for f in goal]), delta

end
function urban_test_all()
    vals = []
    for i in 1:4
        c=Dict{Symbol, Float32}()
        c[:o1] = 3
        c[:o2] = 1
        c[:o3] = 1
        c[:o4] = 1
        c[:o5] = 2
        val=test_lmcut_urban(1; goal_num=i, c=c)
        println("problem 1 goal $i lmcut = $val")
        push!(vals, val)
    end

    
    for i in 1:4
        c=Dict{Symbol, Float32}()
        c[:o1] = 3
        c[:o2] = 1
        c[:o3] = 1
        c[:o4] = 1
        c[:o5] = 2
        val=test_lmcut_urban(2; goal_num=i, c=c)
        println("problem 2 goal $i lmcut = $val")
        push!(vals, val)
    end

    for i in 1:4
        c=Dict{Symbol, Float32}()
        c[:o1] = 3
        c[:o2] = 1
        c[:o3] = 1
        c[:o4] = 2
        c[:o5] = 1
        val=test_lmcut_urban(3; goal_num=i, c=c)
        println("problem 3 goal $i lmcut = $val")
        push!(vals, val)
    end

    c=Dict{Symbol, Float32}()
    c[:o1] = 1
    c[:o2] = 2
    c[:o3] = 1
    c[:o4] = 3
    c[:o5] = 1

    val=test_lmcut_urban(4; c=c)
    println("problem 4 goal 1 lmcut = $val")
    push!(vals, val)

    c=Dict{Symbol, Float32}()
    c[:o1] = 1
    c[:o2] = 2
    c[:o3] = 1
    c[:o4] = 2
    c[:o5] = 1

    val=test_lmcut_urban(5; c=c)
    println("problem 5 goal 1 lmcut = $val")
    push!(vals, val)

    return vals
end

@test urban_test_all() == [3,1,2,3,3,1,2,3,3,1,2,3,3,5]

end