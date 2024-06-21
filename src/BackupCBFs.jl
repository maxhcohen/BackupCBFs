module BackupCBFs

# Required modules
using LinearAlgebra
using DifferentialEquations
using ForwardDiff
using JuMP
using OSQP
using MatrixEquations
using Plots

# Export functionality
export ControlAffineSystem
export linearize
export lqr
export ImplicitCBF
export ImplicitCBFQP
export ImplicitRegulationFilter
export plot_vector_field
export plot_vector_field!

# Include source code
include("systems.jl")
include("linearization.jl")
include("implicit_cbf.jl")
include("implicit_cbf_qp.jl")
include("implicit_regulation_filter.jl")
include("plot_utils.jl")

end # module BackupCBFs
