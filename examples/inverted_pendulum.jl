using Revise
using LinearAlgebra
using BackupCBFs
using Plots

# Dynamics: inverted pendulum
f(x) = [x[2], sin(x[1])]
g(x) = [0.0, 1.0]
Σ = ControlAffineSystem(2, 1, f, g)

# Safety constraint: limit position and velocity of pendulum
θmax = 0.5
θ̇max = 1.0
hs(x) = min(θmax^2 - x[1]^2, θ̇max^2 - x[2]^2)

# Maximum control input
umax = 1.0

# Backup policy: LQR controller about origin
P, K = lqr(Σ, I, I, zeros(2), 0.0)
kb(x) = clamp(-dot(K,x), -umax, umax)

# Backup safe set: some small sublevel set of LQR value function
c = 0.05
V(x) = x'*P*x
hb(x) = c - V(x)

# Plot constraint set and backup set
begin
    fig = contour(range(-0.6, 0.6, 100), range(-1.5, 1.5, 150), (x,y) -> hb([x,y]), levels=[0.0], c=4, colorbar=false, lw=4)
    vline!([-θmax, θmax], c=:black)
    xlabel!(raw"$\theta$")
    ylabel!(raw"$\dot{\theta}$")
end

# Backup horizon and timestep
T = 5.0
dt = 0.1

# Get implicit CBF
h = ImplicitCBF(Σ, hs, hb, kb, T, dt)

# Plot implicit safe set
contour!(fig, range(-0.6, 0.6, 100), range(-1.5, 1.5, 150), (x,y) -> h([x,y]), levels=[0.0], c=1, colorbar=false, lw=4)

# Construct QP-based controller
kd(x) = 0.0
kQP = ImplicitCBFQP(h, kd, -umax, umax)

# Closed-loop dynamics
fcl(x) = f(x) + g(x)*kQP(x)

# Plot vector field
plot_vector_field!(range(-0.6, 0.6, 15), range(-1.5, 1.5, 15), fcl, 0.05; lw=1, c=2)