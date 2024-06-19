using Revise
using LinearAlgebra
using BackupCBFs
using Plots
using ForwardDiff
using DifferentialEquations

# Dynamics
f(x) = [x[3:4]; zeros(2)]
g(x) = [zeros(2,2); diagm(ones(2))]
Σ = ControlAffineSystem(4, 2, f, g)

# Maximum control input
umax = 2.0

# Safety constraint
xo = [-1.0, 1.0]
ro = 0.4
hs(x) = norm(x[1:2] - xo)^2 - ro^2

# Backup policy: stop
Kb = 5.0
kb(x) = clamp.(-Kb*x[3:4], -umax, umax)

# Backup set: some small velocity states
hb(x) = min(0.2^2 - x[3]^2, 0.2^2 - x[4]^2)

# Backup horizon and timestep
T = 5.0
dt = 0.1

# Get implicit CBF
h = ImplicitCBF(Σ, hs, hb, kb, T, dt)
h(rand(4))

# Construct QP-based controller
P, K = lqr(Σ, I, I, zeros(4), zeros(2))
kd(x) = -K*x
kQP = ImplicitCBFQP(h, kd, -umax*ones(2), umax*ones(2))

# Sim params
x0 = [-2.1, 2.0, 0.0, 0.0]
# Closed-loop dynamics
fcl(x) = f(x) + g(x)*kQP(x)

# Sim
sol = solve(ODEProblem((x,p,t) -> fcl(x), x0, (0.0, 10.0)))

# Plot results
plot(sol, idxs=(1,2), label="", xlabel=raw"$x_1$", ylabel=raw"$x_2$")
plot(sol)

# Plot input
ts = 0.0:0.01:T
us = [kQP(sol(t)) for t in ts]
plot(ts, [u[1] for u in us])
plot!(ts, [u[2] for u in us])