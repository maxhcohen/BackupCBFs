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

# Timestep for backup trajectory
dt = 0.1

# Plot constraint set and backup set
function plot_backup_set()
    fig = myplot()
    contour!(range(-0.6, 0.6, 100), range(-1.5, 1.5, 150), (x,y) -> hb([x,y]), levels=[0.0], c=4, colorbar=false, lw=4)
    vline!([-θmax, θmax], c=:black, label="", lw=2)
    xlabel!(raw"$\theta$")
    ylabel!(raw"$\dot{\theta}$")

    return fig
end

# Function for plotting implicit safe set
function plot_implicit_set!(T::Float64)
    h = ImplicitCBF(Σ, hs, hb, kb, T, dt)
    contour!(range(-0.6, 0.6, 100), range(-1.5, 1.5, 150), (x,y) -> h([x,y]), levels=[0.0], c=1, colorbar=false, lw=4)
end

ts = 0.0:0.1:4.0

anim = Animation()
for t in ts
    fig = plot_backup_set()
    plot_implicit_set!(t)
    title!("Backup Horizon: "*raw"$T=$"*"$t")
    frame(anim, fig)
end
# gif(anim, "backup_animation.gif", fps=10)