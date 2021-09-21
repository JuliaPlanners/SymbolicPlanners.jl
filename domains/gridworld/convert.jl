# Functions for generating gridworld PDDL problems
using PDDL

"Converts ASCII gridworlds to PDDL problem."
function ascii_to_pddl(str::String, name="gridworld-problem")
    rows = split(str, "\n", keepempty=false)
    width, height = maximum(length.(strip.(rows))), length(rows)
    wallgrid = parse_pddl("(= wallgrid (new-bit-matrix false $width $height))")
    init = Term[wallgrid]
    start, goal = Term[], pddl"(true)"
    for (y, row) in enumerate(rows)
        for (x, char) in enumerate(strip(row))
            if char == '.' # Unoccupied
                continue
            elseif char == 'W' # Wall
                wall = parse_pddl("(= wallgrid (set-index wallgrid true $y $x))")
                push!(init, wall)
            elseif char == 's' # Start position
                start = parse_pddl("(= xpos $x)", "(= ypos $y)")
            elseif char == 'g' # Goal position
                goal = parse_pddl("(and (= xpos $x) (= ypos $y))")
            end
        end
    end
    append!(init, start)
    problem = GenericProblem(
        Symbol(name), :gridworld, Const[], Dict{Const,Symbol}(),
        init, goal, nothing)
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
