"""
    nonNaNmax(A)

Finds maximum element != NaN in array
"""
function nonNaNmax(A)
    Amax = maximum(x->isnan(x) ? -Inf : x,A)
    idxMax = findfirst(==(Amax), A)
    return A[idxMax], idxMax
end

"""
    nonNaNmin(A)

Finds minimum element != NaN in array
"""
function nonNaNmin(A)
    Amin = minimum(x->isnan(x) ? Inf : x,A)
    idxMin = findfirst(==(Amin), A)
    return A[idxMin], idxMin
end
"""
     workspace_conditioning(wg::Vector; solution::Vector{Int}, res::Int = 200)

Computes the sum of the condition indices of the workspace (-π, π) for a given resolution 'res'
"""
function workspace_conditioning(wg::Vector; solution::Vector{Int}, res::Int = 200)
    wscond = 0.
    # sampling over entire workspace
    for i in range(-π, π, length = res)
        for j in range(-π, π, length = res)
            J = constraint_jacobian([i,j], wg, solution = solution)
            wscond[i,j] = 1/cond(Matrix{Real}(J*J'))
        end
    end
    return wscond
end

"""
    mynorm(vec::Vector)

Additional function to stupidly computing the norm of a vector
"""
function mynorm(vec::Vector)
    return √(sum([v^2 for v in vec]))
end

function Base.:/(vec::Vector{NonlinearExpr}, a::NonlinearExpr)
    return [v/a for v in vec]
end