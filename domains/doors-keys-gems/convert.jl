# Functions for generating gridworld PDDL problems
using PDDL

"Converts ASCII gridworlds to PDDL problem."
function ascii_to_pddl(str::String, name="doors-keys-gems-problem")
    rows = split(str, "\n", keepempty=false)
    width, height = maximum(length.(strip.(rows))), length(rows)
    doors, keys, gems = Const[], Const[], Const[]
    walls = parse_pddl("(= walls (new-bit-matrix false $width $height))")
    init = Term[walls]
    start, goal = Term[], pddl"(true)"
    for (y, row) in enumerate(rows)
        for (x, char) in enumerate(strip(row))
            if char == '.' # Unoccupied
                continue
            elseif char == 'W' # Wall
                wall = parse_pddl("(= walls (set-index walls true $y $x))")
                push!(init, wall)
            elseif char == 'D' # Wall
                d = Const(Symbol("door$(length(doors)+1)"))
                push!(doors, d)
                append!(init, parse_pddl("(= (xloc $d) $x)", "(= (yloc $d) $y)"))
                push!(init, parse_pddl("(locked $d)"))
            elseif char == 'k' # Key
                k = Const(Symbol("key$(length(keys)+1)"))
                push!(keys, k)
                append!(init, parse_pddl("(= (xloc $k) $x)", "(= (yloc $k) $y)"))
            elseif char == 'g' || char == 'G' # Gem
                g = Const(Symbol("gem$(length(gems)+1)"))
                push!(gems, g)
                append!(init, parse_pddl("(= (xloc $g) $x)", "(= (yloc $g) $y)"))
                if char == 'G' goal = parse_pddl("(has $g)") end
            elseif char == 's' # Start position
                start = parse_pddl("(= xpos $x)", "(= ypos $y)")
            end
        end
    end
    append!(init, start)
    objtypes = merge(Dict(d => :door for d in doors),
                     Dict(k => :key for k in keys),
                     Dict(g => :gem for g in gems))
    problem = GenericProblem(Symbol(name), Symbol("doors-keys-gems"),
                             [doors; keys; gems], objtypes, init, goal, nothing)
    return problem
end

function load_ascii_problem(path::AbstractString)
    str = open(f->read(f, String), path)
    return ascii_to_pddl(str)
end

function convert_ascii_problem(path::String)
    str = open(f->read(f, String), path)
    str = ascii_to_pddl(str)
    new_path = join(split(path, ".")[1:end-1]) * ".pddl"
    write(new_path, write_problem(str))
    return new_path
end
