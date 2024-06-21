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
hb(x) = min(0.5^2 - x[3]^2, 0.5^2 - x[4]^2)

# Backup horizon and timestep
Tb = 5.0
dt = 0.1

# Get implicit CBF
h = ImplicitCBF(Σ, hs, hb, kb, Tb, dt)

# Construct QP-based controller for getting to goal
P, K = lqr(Σ, diagm(ones(4)), diagm(ones(2)), zeros(4), zeros(2))
kd(x) = -K*x

# Safety filter
kQP = ImplicitCBFQP(h, kd, -umax*ones(2), umax*ones(2))
kQP(zeros(4))

# Sim params
T = 10.0
x0 = [-2.1, 2.0, 0.0, 0.0]

# Closed-loop dynamics
fcl(x) = f(x) + g(x)*kQP(x)

# Sim
sol = solve(ODEProblem((x,p,t) -> fcl(x), x0, (0.0, T)))

# Plot results
begin
    plot(sol, idxs=(1,2), label="", xlabel=raw"$x_1$", ylabel=raw"$x_2$")
    contour!(range(-2.0, 0.0, 100), range(0.0, 2.0, 100), (x,y) -> hs([x,y]), c=:black,lw=3,colorbar=false,levels=[0]) 
end

begin
    plot(sol, label=[raw"$x_1$" raw"$x_2$" raw"$x_3$" raw"$x_4$"], xlabel=raw"$t$", ylabel=raw"$x(t)$", xlims=(0.0,T))
end

# Plot input
ts = 0.0:0.01:T
us = [kQP(sol(t)) for t in ts]
begin
    plot(ts, [u[1] for u in us], label=raw"$u_1$")
    plot!(ts, [u[2] for u in us], label=raw"$u_2$") 
    hline!([-umax, umax], c=:black, ls=:dash)
    xlims!(0.0, T)
    xlabel!(raw"$t$")
    ylabel!(raw"$u(t)$")
end