using ConstructionBase: setproperties

"""
    $(SIGNATURES)

Returns `true` if the specification wrapes around a sub-specification.
"""
has_subspec(spec::Specification) = false

"""
    $(SIGNATURES)

Returns the sub-specification of a wrapper specification.
"""
get_subspec(spec::Specification) = error("Not implemented.")

"""
    $(SIGNATURES)

Sets the sub-specification of a wrapper specification.
"""
set_subspec(spec::Specification, subspec) = error("Not implemented.")

"""
    @set_subspec spectype field

Forwards all undefined [`Specification`] interface methods for a `spectype` to
the sub-specification at `field`.
"""
macro set_subspec(spectype, field::Symbol)
    # Escaped function names
    _has_subspec = GlobalRef(SymbolicPlanners, :has_subspec)
    _get_subspec = GlobalRef(SymbolicPlanners, :get_subspec)
    _set_subspec = GlobalRef(SymbolicPlanners, :set_subspec)
    _is_goal = GlobalRef(SymbolicPlanners, :is_goal)
    _is_violated = GlobalRef(SymbolicPlanners, :is_violated)
    _get_cost = GlobalRef(SymbolicPlanners, :get_cost)
    _get_reward = GlobalRef(SymbolicPlanners, :get_reward)
    _get_discount = GlobalRef(SymbolicPlanners, :get_discount)
    _get_goal_terms = GlobalRef(SymbolicPlanners, :get_goal_terms)
    _set_goal_terms = GlobalRef(SymbolicPlanners, :set_goal_terms)
    _has_action_goal = GlobalRef(SymbolicPlanners, :has_action_goal)
    _has_action_cost = GlobalRef(SymbolicPlanners, :has_action_cost)
    _get_action_cost = GlobalRef(SymbolicPlanners, :get_action_cost)
    # Escaped variable names
    _spec = esc(:spec)
    _subspec = :($_spec.$field)
    _domain = esc(:domain)
    _state = esc(:state)
    _action = esc(:action)
    _s1 = esc(:s1)
    _a = esc(:a)
    _s2 = esc(:s2)
    _terms = esc(:terms)
    # Generate method definitions
    expr = quote
        $(_has_subspec)($_spec::$(esc(spectype))) = true
        $(_get_subspec)($_spec::$(esc(spectype))) = $_subspec
        function $(_set_subspec)($_spec::$(esc(spectype)), $(esc(:subspec)))
            return setproperties(spec, ($(field)=$(esc(:subspec)),))
        end
        if !_spec_hasmethod($(_is_goal), $(esc(spectype)), Domain, State, Term)
            $(_is_goal)($_spec::$(esc(spectype)), $_domain::Domain, $_state::State, $_action::Term) =
                $(_is_goal)($_subspec, $_domain, $_state, $_action)
        end
        if !_spec_hasmethod($(_is_goal), $(esc(spectype)), Domain, State, Nothing)
            $(_is_goal)($_spec::$(esc(spectype)), $_domain::Domain, $_state::State, $_action::Nothing) =
                $(_is_goal)($_subspec, $_domain, $_state, $_action)
        end
        if !_spec_hasmethod($(_is_goal), $(esc(spectype)), Domain, State)
            $(_is_goal)($_spec::$(esc(spectype)), $_domain::Domain, $_state::State) =
                $(_is_goal)($_subspec, $_domain, $_state)
        end
        if !_spec_hasmethod($(_is_violated), $(esc(spectype)), Domain, State)
            $(_is_violated)($_spec::$(esc(spectype)), $_domain::Domain, $_state::State) =
                $(_is_violated)($_subspec, $_domain, $_state)
        end
        if !_spec_hasmethod($(_get_cost), $(esc(spectype)), Domain, State, Term, State)
            $(_get_cost)($_spec::$(esc(spectype)), $_domain::Domain, $_s1::State, $_a::Term, $_s2::State) =
                $(_get_cost)($_subspec, $_domain, $_s1, $_a, $_s2)
        end
        if !_spec_hasmethod($(_get_reward), $(esc(spectype)), Domain, State, Term, State)
            $(_get_reward)($_spec::$(esc(spectype)), $_domain::Domain, $_s1::State, $_a::Term, $_s2::State) =
                $(_get_reward)($_subspec, $_domain, $_s1, $_a, $_s2)
        end
        if !_spec_hasmethod($(_get_discount), $(esc(spectype)))
            $(_get_discount)($_spec::$(esc(spectype))) =
                $(_get_discount)($_subspec)
        end
        if !_spec_hasmethod($(_get_goal_terms), $(esc(spectype)))
            $(_get_goal_terms)($_spec::$(esc(spectype))) =
                $(_get_goal_terms)($_subspec)
        end
        if !_spec_hasmethod($(_set_goal_terms), $(esc(spectype)), Any)
            function $(_set_goal_terms)($_spec::$(esc(spectype)), $_terms)
                $(esc(:subspec)) = $(_set_goal_terms)($_subspec, $_terms)
                return setproperties(spec, ($(field)=$(esc(:subspec)),))
            end
        end
        if !_spec_hasmethod($(_has_action_goal), $(esc(spectype)))
            $(_has_action_goal)($_spec::$(esc(spectype))) =
                $(_has_action_goal)($_subspec)
        end
        if !_spec_hasmethod($(_has_action_cost), $(esc(spectype)))
            $(_has_action_cost)($_spec::$(esc(spectype))) =
                $(_has_action_cost)($_subspec)
        end
        if !_spec_hasmethod($(_get_action_cost), $(esc(spectype)), Term)
            $(_get_action_cost)($_spec::$(esc(spectype)), $_action::Term) =
                $(_get_action_cost)($_subspec, $_action)
        end
    end
    return expr
end

function _spec_hasmethod(f, spectype, argtypes...)
    if hasmethod(f, Tuple{spectype, argtypes...})
        method = Base.which(f, Tuple{spectype, argtypes...})
        return method.sig <: Tuple{typeof(f), spectype, argtypes...}
    else
        return false
    end
end
