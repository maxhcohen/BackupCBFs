"""
    ImplicitCBF

CBF implicitly defined by the flow of a safe backup controller.

# Fields
- `h::Function` : Function whose zero superlevel set defines the implicit safe set
- `grad::Function` : Gradient of the implicit CBF
- `Lfh::Function` : Lie derivative of h along drift vector field
- `Lgh::Function` : Lie derivative of h along control directions
"""
struct ImplicitCBF
    h::Function
    grad::Function
    Lfh::Function
    Lgh::Function
end

"""
    (h::ImplicitCBF)(x)

Evaluate implicit CBF at current state
"""
(h::ImplicitCBF)(x) = h.h(x)

"""
    ImplicitCBF(Σ::ControlAffineSystem, h::Function, hb::Function, kb::Function, T::Float64, dt::Float64)

Construct an implicit CBF from a system, safety constraint, and backup policy.

# Arguments
- `Σ::ControlAffineSystem` : control system under consideration
- `h::Function` : function whose zero superlevel set defines the safety constraint
- `hb::Function` : function whose zero superlevel set defines the backup safe set
- `kb::Function` : backup controller
- `T::Float64` : time horizon used to compute flow under backup policy
- `dt::Float64` : timestep used to discetize backup horizon
"""
function ImplicitCBF(Σ::ControlAffineSystem, h::Function, hb::Function, kb::Function, T::Float64, dt::Float64)
    # Pull out dynamics
    n, m, f, g = Σ.n, Σ.m, Σ.f, Σ.g

    # Closed-loop vector field under backup policy
    fb(x) = f(x) + g(x)*kb(x)

    # DiffEq problem for computing flow
    function odefun!(dx, x, p, t)
        dx[1:n] .= fb(x)
    end

    # Discretization of time horizon
    ts = 0.0:dt:T

    # Implicit safe set
    function hI(x)
        sol = solve(ODEProblem(odefun!, x, (0.0,T)))
    
        return minimum(vcat(h.(sol.(ts)), hb(sol(T))))
    end

    # Compute gradients and Lie derivtives so we don't have to later
    ∇hI(x) = ForwardDiff.gradient(hI, x)
    LfhI(x) = ∇hI(x)'*f(x)
    LghI(x) = ∇hI(x)'*g(x)

    return ImplicitCBF(hI, ∇hI, LfhI, LghI)
end