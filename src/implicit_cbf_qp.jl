
"""
    ImplicitCBFQP

CBF-QP corresponding to implicit CBF
"""
struct ImplicitCBFQP
    get_input::Function
end
(k::ImplicitCBFQP)(x) = k.get_input(x)

"""
    ImplicitCBFQP(h::ImplicitCBF, kd::Function, umin::Float64, umax::Float64)
    ImplicitCBFQP(h::ImplicitCBF, kd::Function, umin::Vector{Float64}, umax::Vector{Float64})

Construct an implicit safety filter from and implicit CBF, a desired controller, and input bounds.

# Arguments
- `h::ImplicitCBF` : the implicit CBF
- `kd::Function` : desired control policy we wish to pass through the safety filter
- `umin::Union{Float64, Vector{Float64}}` : lower bound on control input u ≥ umin
- `umax::Union{Float64, Vector{Float64}}` : upper bound on control input u ≤ umax
"""
function ImplicitCBFQP(h::ImplicitCBF, kd::Function, umin::Float64, umax::Float64)
    # Lie derivatives
    Lfh, Lgh = h.Lfh, h.Lgh

    # CBF-QP
    function k(x)
        model = Model(OSQP.Optimizer)
        set_silent(model)
        @variable(model, umin ≤ u ≤ umax)
        @variable(model, α)
        @objective(model, Min, 0.5*(u - kd(x))^2 + (α - 1.0)^2)
        @constraint(model, Lfh(x) + Lgh(x)*u ≥ -α*h(x))
        optimize!(model)
    
        return value(u)
    end

    return ImplicitCBFQP(k)
end

function ImplicitCBFQP(h::ImplicitCBF, kd::Function, umin::Vector{Float64}, umax::Vector{Float64})
    # Lie derivatives
    Lfh, Lgh = h.Lfh, h.Lgh
    m = length(umax)

    # CBF-QP
    function k(x)
        model = Model(OSQP.Optimizer)
        set_silent(model)
        @variable(model, u[1:m])
        @variable(model, α)
        @objective(model, Min, 0.5*(u - kd(x))'*(u - kd(x)) + (α - 1.0)^2)
        @constraint(model, Lfh(x) + Lgh(x)*u ≥ -α*h(x))
        @constraint(model, u .≤ umax)
        @constraint(model, u .≥ umin)
        optimize!(model)
    
        return value.(u)
    end

    return ImplicitCBFQP(k)
end