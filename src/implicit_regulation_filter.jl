struct ImplicitRegulationFilter
    get_input::Function
end
(k::ImplicitRegulationFilter)(x) = k.get_input(x)

function ImplicitRegulationFilter(h::ImplicitCBF, kb::Function, kd::Function, β::Float64)
    λ(h) = 1.0 - exp(-β*max(0.0, minimum(h)))
    k(x) = λ(h(x))*kd(x) + (1.0 - λ(h(x)))*kb(x)

    return ImplicitRegulationFilter(k)
end