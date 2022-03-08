"""
    nonNaNmax(A)

Finds maximum element != NaN in array
"""
function nonNaNmax(A)
    Amax = maximum(x->isnan(x) ? -Inf : x,A)
    idxMax = findfirst(==(Amax), A)
    return A[idxMax], idxMax # either change nonNaNmin to also return idx or only return index
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
