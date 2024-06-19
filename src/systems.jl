"""
    ControlAffineSystem

Data structure representing a control affine system.

# Fields
- `n::Int` : State dimension
- `m::Int` : Input dimension
- `f::Function` : Drift dynamics
- `g::Function` : Control directions
"""
struct ControlAffineSystem
    n::Int
    m::Int
    f::Function
    g::Function
end

# Some example systems
function InvertedPendulum()
    f(x) = [x[2], sin(x[1])]
    g(x) = [0.0, 1.0]

    return ControlAffineSystem(2, 1, f, g)
end

function DoubleIntegrator()
    f(x) = [x[3:4]; zeros(2)]
    g(x) = [zeros(2,2); diagm(ones(2))]

    return ControlAffineSystem(4, 2, f, g)
end