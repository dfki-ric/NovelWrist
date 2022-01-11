"""
Type to carry over the geometric properties of a mechanism, both sides must be
defined.
"""
mutable struct WristGeometry
    l::Tuple{Real, Real}
    r::Tuple{Real, Real}
    r_::Tuple{Real, Real}
    h::Tuple{Real, Real}
    b::Tuple{Vector{<:Real}, Vector{<:Real}}
    c::Tuple{Vector{<:Real}, Vector{<:Real}}
    e0::Tuple{Vector{<:Real}, Vector{<:Real}}
    n::Tuple{Vector{<:Real}, Vector{<:Real}}
    actuator_limits::Tuple{Tuple{Real, Real}, Tuple{Real, Real}}

    function WristGeometry(;l::Tuple{Real, Real},
                            r::Tuple{Real, Real},
                            r_::Tuple{Real, Real},
                            h::Tuple{Real, Real},
                            b::Tuple{Vector{<:Real}, Vector{<:Real}},
                            c::Tuple{Vector{<:Real}, Vector{<:Real}},
                            e0::Tuple{Vector{<:Real}, Vector{<:Real}},
                            n::Tuple{Vector{<:Real}, Vector{<:Real}},
                            actuator_limits::Tuple{Tuple{Real, Real}, Tuple{Real, Real}})

    @assert all(vcat([norm(ni) for ni in n] .< 1 + 1e-5, [norm(ni) for ni in n] .> 1 - 1e-5)) "Normal vectors not of unity!"

        return new(l, r, r_, h, b, c, e0, n, actuator_limits)
    end
end
