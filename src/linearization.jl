"""
    linearize(Σ::ControlAffineSystem, x0, u0)

Linearize a control affine system about state `x0` and input `u0`.
"""
function linearize(Σ::ControlAffineSystem, x0, u0)
    f(x,u) = Σ.f(x) + Σ.g(x)*u
    A = ForwardDiff.jacobian(x -> f(x, u0), x0)
    B = Σ.m == 1 ? ForwardDiff.derivative(u -> f(x0, u), u0) : ForwardDiff.jacobian(u -> f(x0, u), u0)

    return A, B
end

"""
    lqr(A, B, Q, R)
    lqr(A, B, Q, R::Float64)
    lqr(Σ::ControlAffineSystem, Q, R, x0, u0)

Compute an LQR controller and corresponding Lyapunov function for a linear system.

# Returns
- `P` : The solution to the Algebraic Ricatti Equation
- `K` : The gain of the LQR controller
"""
function lqr(A, B, Q, R)
    P, _, K = arec(A, B, R, Q)

    return P, K
end

function lqr(A, B, Q, R::Float64)
    P, _, K = arec(A, B, R*ones(1,1), Q)

    return P, K
end

function lqr(Σ::ControlAffineSystem, Q, R, x0, u0)
    A, B = linearize(Σ, x0, u0)

    return lqr(A, B, Q, R)
end