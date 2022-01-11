"""
General rotation matrices
"""
function Rx(ϕ::Real)
    Rx = [1 0 0; 0 cos(ϕ) -sin(ϕ); 0 sin(ϕ) cos(ϕ)]
end

function Ry(ϕ::Real)
    Ry = [cos(ϕ) 0 sin(ϕ); 0 1 0; -sin(ϕ) 0 cos(ϕ)]
end

function Rz(ϕ::Real)
    Rz = [cos(ϕ) -sin(ϕ) 0; sin(ϕ) cos(ϕ) 0; 0 0 1]
end

"""
Returns the intersection points that arise from the problem of circle-sphere
intersection
"""
function circle_sphere_intersection(s::Vector{<:Real}, t::Real, cp::Vector{<:Real}, n::Vector{<:Real}, r::Real)

    # computing projecting vector
    η = dot((cp - s), n)*n
    # distance between both circles
    δ = s + η - cp
    # reducing length to center line
    δp = δ*(0.5 + (r^2 - t^2 + dot(η, η))/2dot(δ, δ))
    # and vector orthogonal to that
    if dot(δp, δp) > r^2
        p = [NaN, NaN, NaN]
    else
        p = cross(δp/norm(δp), n*√(r^2 - dot(δp, δp)))
    end

    return (cp + δp + p, cp + δp - p)
end

"""
Retrieving inverse kinematics solution from point locations and returns the
location of point k to avoid multiple evaluation
"""
function inverse_kinematics(x::Vector{<:Real}, wg::WristGeometry; specsol::Vector{Int}, intrinsic::Bool)

    @unpack l, r, r_, h, b, c, e0, n = wg
    q = Vector{Real}(undef, 2)

    # determine rotation
    if intrinsic
        R = Rz(x[2])*Rx(x[1])
    else
        R = Rz(x[1])*Rx(x[2])
    end

    for i = 1:2
        # end-effector rotation
        e = R*e0[i]
        # projected circle centre
        cp = c[i] + h[i]*n[i]
        # computing intersection point
        k = circle_sphere_intersection(e, l[i], cp, n[i], r[i])
        # expressing vector difference
        vt = c[i] + r_[i]/r[i]*(k[specsol[i]] - c[i] - h[i]*n[i]) - b[i]
        # distance of actuator
        q[i] = √dot(vt, vt)
    end

    return q
end

"""
Retrieving inverse kinematics solution from point locations and returns the
location of point k to avoid multiple evaluation
"""
function forward_kinematics(q::Vector{<:Real}, wg::WristGeometry; specsol::Vector{Int})

    @unpack l, r, r_, h, b, c, e0, n = wg
    x = Vector{Real}(undef, 2)
    k = Vector{Vector{Real}}(undef, 2)

    for i = 1:2
        # computing intersection point
        m = circle_sphere_intersection(b[i], q[i], c[i], n[i], r_[i])
        # location of crank points
        k[i] = c[i] + r[i]/r_[i]*(m[specsol[i]] - c[i]) + h[i]*n[i]
    end

    return k
end

"""
Constraint equation depending on x and q
"""
function constraints(x::Vector, q::Vector, wg::WristGeometry, specsol::Vector{Int}, intrinsic::Bool)
    @unpack l, r, r_, h, b, c, e0, n = wg
    con = Vector{Real}(undef, 2)

    # determine rotation
    if intrinsic
        R = Rz(x[2])*Rx(x[1])
    else
        R = Rz(x[1])*Rx(x[2])
    end

    for i = 1:2
        # end-effector rotation
        e = R*e0[i]
        # projected circle centre
        cp = c[i] + h[i]*n[i]
        # computing intersection point
        k = circle_sphere_intersection(e, l[i], cp, n[i], r[i])
        # expressing vector difference
        vt = c[i] + r_[i]/r[i]*(k[specsol[i]] - c[i] - h[i]*n[i]) - b[i]
        # constraint
        con[i] = q[i]^2 - dot(vt, vt)
    end

    return con
end

"""
Differential kinematics of the constraints
"""
function Jacobian(x::Vector{<:Real}, wg::WristGeometry; specsol::Vector{Int}, intrinsic::Bool, split::Bool = false)

    q = inverse_kinematics(x, wg, specsol = specsol, intrinsic = intrinsic)

    # differentiation by autodiff
    Jx = ForwardDiff.jacobian(x_ -> constraints(x_, q, wg, specsol, intrinsic), x)
    Jq = ForwardDiff.jacobian(q_ -> constraints(x, q_, wg, specsol, intrinsic), q)

    if split
        return Jx, Jq
    else
        return Matrix{Real}(-inv(Jx)*Jq)
    end
end

"""
Inverse kinematics of the comparative 2UPS + 1U mechanism - simply computing
Euclidean distance
"""
function inverse_kinematics_C(x::Vector{<:Real}, wg::WristGeometry; intrinsic::Bool)

    @unpack b, e0 = wg
    q = Vector{Real}(undef, 2)

    # determine rotation
    if intrinsic
        R = Rz(x[2])*Rx(x[1])
    else
        R = Rz(x[1])*Rx(x[2])
    end

    for i = 1:2
        # end-effector rotation
        e = R*e0[i]
        # distance of actuator
        q[i] = norm(e - b[i])
    end

    return q
end

"""
Comparative 2UPS + 1U mechanism constraint equation depending on x and q
"""
function constraints_C(x::Vector, q::Vector, wg::WristGeometry, intrinsic::Bool)
    @unpack b, e0 = wg
    con = Vector{Real}(undef, 2)

    # determine rotation
    if intrinsic
        R = Rz(x[2])*Rx(x[1])
    else
        R = Rz(x[1])*Rx(x[2])
    end

    for i = 1:2
        # end-effector rotation
        e = R*e0[i]
        # constraint
        con[i] = q[i]^2 - dot(e - b[i], e - b[i])
    end

    return con
end

"""
Differential kinematics of the constraints of comparative 2UPS + 1U mechanism
"""
function Jacobian_C(x::Vector{<:Real}, wg::WristGeometry; intrinsic::Bool, split::Bool = false)

    q = inverse_kinematics_C(x, wg, intrinsic = intrinsic)

    # differentiation by autodiff
    Jx = ForwardDiff.jacobian(x_ -> constraints_C(x_, q, wg, intrinsic), x)
    Jq = ForwardDiff.jacobian(q_ -> constraints_C(x, q_, wg, intrinsic), q)

    if split
        return Jx, Jq
    else
        return Matrix{Real}(-inv(Jx)*Jq)
    end
end