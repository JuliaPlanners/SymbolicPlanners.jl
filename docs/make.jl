using Documenter, SymbolicPlanners

makedocs(
   sitename="SymbolicPlanners.jl",
   format=Documenter.HTML(
      prettyurls=get(ENV, "CI", nothing) == "true",
      canonical="https://juliaplanners.github.io/SymbolicPlanners.jl/stable",
      description="Documentation for SymbolicPlanners.jl, " *
                  "a library of planners for PDDL domains.",
      assets = ["assets/logo.ico"],
      highlights=["lisp"] # Use Lisp highlighting as PDDL substitute
   ),
   pages=[
      "Home" => "index.md",
      "Planners" => "planners.md",
      "Heuristics" => "heuristics.md",
      "Specifications" => "specifications.md",
      "Solutions" => "solutions.md",
      "Simulators" => "simulators.md",
   ]
)

deploydocs(
    repo = "github.com/JuliaPlanners/SymbolicPlanners.jl.git",
    forcepush = true
)
