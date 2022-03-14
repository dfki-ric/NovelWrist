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

"""
Wrist build in https://robotik.dfki-bremen.de/de/forschung/robotersysteme/rh5-manus/
"""
RH5_wrist = WristGeometry(l = (0.045, 0.045), 
                          r = (0.049, 0.049), 
                          r_ = (0.049, 0.049),
                          h = (0.012, 0.012),
                          b = ([0.015, -0.178, -0.034], [-0.015, -0.178, -0.034]),
                          c = ([0.015, -0.032, 0.011], [-0.015, -0.032, 0.011]),
                          e0 = ([0.027, 0, -0.030], [-0.027, 0, -0.030]),
                          n = ([1, 0, 0], [1, 0, 0]),
                          actuator_limits = ((0.113, 0.178), (0.113, 0.178)));
