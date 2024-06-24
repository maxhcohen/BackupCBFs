"""
    plot_vector_field(xs, ys, f::Function, scale::Float64; kwargs...)

Plot a vector field characterized by `f` onto a new plot.

# Arguments
- `xs` : list of x coordinates used to grid for vector field
- `ys` : list of y coordinates used to grid for vector field
- `f::Function` : function `f(x)` describing the vector field
- `scale::Float64` : how much to scale the vectors by

# Keyword Arugments: Anything you can toss to quiver.
"""
function plot_vector_field(xs, ys, f::Function, scale::Float64; kwargs...)
    X = [x for x in xs for y in ys]
    Y = [y for x in xs for y in ys]
    Xs = [[x,y] for (x,y) in zip(X,Y)]
    f1 = [scale*normalize(f(x))[1] for x in Xs]
    f2 = [scale*normalize(f(x))[2] for x in Xs]
    quiver(X, Y, quiver=(f1, f2); kwargs...)
end

"""
    plot_vector_field!(xs, ys, f::Function, scale::Float64; kwargs...)

Plot a vector field characterized by `f` onto an existing plot.

# Arguments
- `xs` : list of x coordinates used to grid for vector field
- `ys` : list of y coordinates used to grid for vector field
- `f::Function` : function `f(x)` describing the vector field
- `scale::Float64` : how much to scale the vectors by

# Keyword Arugments: Anything you can toss to quiver.
"""
function plot_vector_field!(xs, ys, f::Function, scale::Float64; kwargs...)
    X = [x for x in xs for y in ys]
    Y = [y for x in xs for y in ys]
    Xs = [[x,y] for (x,y) in zip(X,Y)]
    f1 = [scale*normalize(f(x))[1] for x in Xs]
    f2 = [scale*normalize(f(x))[2] for x in Xs]
    quiver!(X, Y, quiver=(f1, f2); kwargs...)
end

"""
    myplot()

Make a new plot with some attributes that I like.
"""
myplot() = plot(grid=false, framestyle=:box, palette=:tab10, fontfamily="Computer Modern", guidefontsize=14, tickfontsize=10)