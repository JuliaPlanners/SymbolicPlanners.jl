# Simulators

```@meta
CurrentModule = SymbolicPlanners
```

**SymbolicPlanners.jl** extends the `PDDL.Simulator` interface to enable
simulation of plan or policy [`Solution`](@ref)s:

```@docs
Simulator(::Solution, ::Domain, ::State, ::Specification)
Simulator(::Domain, ::State, ::Any, ::Specification)
```

The following simulators are can be used to simulate solutions:

```@docs
EndStateSimulator
StateRecorder
StateActionRecorder
RewardAccumulator
```