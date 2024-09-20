"""
    Rx(ϕ::Real) / Ry(ϕ::Real) / Rz(ϕ::Real)

General rotation matrices for x/ y/ z axis, respectively
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
    circle_sphere_intersection(s::Vector{<:Real}, t::Real, cp::Vector{<:Real}, n::Vector{<:Real}, r::Real)

Returns the intersection points that arise from the problem of circle-sphere intersection that is given by
the sphere with center 's' and radius 't' and the circle with center 'cp', radius 'r', and the vector normal to the circle plane 'n'.  
"""
function circle_sphere_intersection(s::Vector, t, cp::Vector, n::Vector, r)

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
        p = LinearAlgebra.cross(δp/norm(δp), n*√(r^2 - dot(δp, δp)))
    end

    return (cp + δp + p, cp + δp - p)
end

"""
    inverse_kinematics(x::Vector{<:Real}, wg::WristGeometry; solution::Vector{Int}, intrinsic::Bool)

Retrieving inverse kinematics solution from point end-effector rotation 'x' and returns the
location of point k to avoid multiple evaluation. The solution is specified by 'solution'. 
"""
function inverse_kinematics(x::Vector, wg::Vector; solution::Vector{Int}, intrinsic::Bool = true) 

    @assert length(solution) == 2 && all([s in Set([1,2]) for s in solution]) "Solution vector incorrect!"

   # unpacking the vector onto the geometry variables
   l, r, r_, h = wg[1:4]
   b, c, e0, n = wg[5:8]

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
        vt = c[i] + r_[i]/r[i]*(k[solution[i]] - c[i] - h[i]*n[i]) - b[i]
        # distance of actuator
        q[i] = √dot(vt, vt)
    end

    return q
end

"""
    forward_kinematics(q::Vector{<:Real}, wg::WristGeometry; solution::Vector{Int})

Retrieving the end-effector orientation, given the solution for the actuator lengths 'q'.
"""
function forward_kinematics(q::Vector{<:Real}, wg::Vector; solution::Vector{Int}, intrinsic::Bool = true)

    @assert length(solution) == 3 && all([s in Set([1,2]) for s in solution]) "Solution vector incorrect!"

    # unpacking the vector onto the geometry variables
    l, r, r_, h = wg[1:4]
    b, c, e0, n = wg[5:8]

    k = Vector{Vector{Real}}(undef, 2)
    m = Vector{Vector{Real}}(undef, 2)

    for i = 1:2
        # computing intersection point
        m[i] = circle_sphere_intersection(b[i], q[i], c[i], n[i], r_[i])[solution[i]]
        # location of crank points
        k[i] = c[i] + r[i]/r_[i]*(m[i] - c[i]) + h[i]*n[i]
    end

    Ring, (t, u, v, w) = PolynomialRing(QQ, ["t", "u", "v", "w"], ordering = :degrevlex)

    if intrinsic
        R = [v -w 0; w v 0; 0 0 1]*[1 0 0; 0 t -u; 0 u t]
    else
        R = [t -u 0; u t 0; 0 0 1]*[1 0 0; 0 v -w; 0 w v]
    end

    eq1 = dot(R*Rational.(e0[1]) - Rational.(k[1]), R*Rational.(e0[1]) - Rational.(k[1])) - Rational(l[1]^2)
    eq2 = dot(R*Rational.(e0[2]) - Rational.(k[2]), R*Rational.(e0[2]) - Rational.(k[2])) - Rational(l[2]^2)

    eq3 = t^2 + u^2 - 1
    eq4 = v^2 + w^2 - 1

    II = ideal([eq1, eq2, eq3, eq4])

    _, sol  = msolve(II, info_level = 0);

    return [atan(Float64(sol[solution[3]][2]), Float64(sol[solution[3]][1])), atan(Float64(sol[solution[3]][4]), Float64(sol[solution[3]][3]))]

end


"""
    constraints(x::Vector, q::Vector, wg::WristGeometry, solution::Vector{Int}, intrinsic::Bool)

Constraint equation depending on the end-effector rotation 'x' and actuator lengths 'q'.
"""
function constraints(x::Vector, q::Vector, wg::Vector, solution::Vector{Int}, intrinsic::Bool = true)

    @assert length(solution) == 2 && all([s in Set([1,2]) for s in solution]) "Solution vector incorrect!"

    # unpacking the vector onto the geometry variables
    l, r, r_, h = wg[1:4]
    b, c, e0, n = wg[5:8]

    con = Vector{Any}(undef, 2)

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
        vt = c[i] + r_[i]/r[i]*(k[solution[i]] - c[i] - h[i]*n[i]) - b[i]
        # constraint
        con[i] = q[i]^2 - dot(vt, vt)
    end

    return con
end

"""
    Jacobian(x::Vector{<:Real}, wg::WristGeometry; solution::Vector{Int}, intrinsic::Bool, split::Bool = false)

Differential kinematics of the constraints.
"""
function constraint_jacobian(x::Vector{<:Real}, wg::Vector; solution::Vector{Int}, intrinsic::Bool = true, split::Bool = false)

    @assert length(solution) == 2 && all([s in Set([1,2]) for s in solution]) "Solution vector incorrect!"

    q = inverse_kinematics(x, wg, solution = solution, intrinsic = intrinsic)

    # differentiation by autodiff
    Jx = ForwardDiff.jacobian(x_ -> constraints(x_, q, wg, solution, intrinsic), x)
    Jq = ForwardDiff.jacobian(q_ -> constraints(x, q_, wg, solution, intrinsic), q)

    if split
        return Jx, Jq
    else
        return -inv(Jx)*Jq
    end
end      

"""
    inverse_kinematics_C(x::Vector{<:Real}, wg::WristGeometry; intrinsic::Bool)

Inverse kinematics of the comparative 2UPS + 1U mechanism - simply computing
Euclidean distance given the end-effector orientation 'x'
"""
function inverse_kinematics_2SPU1U(x::Vector{<:Real}, wg::Vector; intrinsic::Bool = true)

    # unpacking the vector onto the geometry variables
    b, e0 = wg[5], wg[7]
    
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
    constraints_C(x::Vector, q::Vector, wg::WristGeometry, intrinsic::Bool)

Comparative 2UPS + 1U mechanism constraint equation depending on end-effector orientation 'x' and actuator lengths 'q'.
"""
function constraints_2SPU1U(x::Vector, q::Vector, wg::Vector, intrinsic::Bool = true)
    
    # unpacking the vector onto the geomtry variables
    b, e0 = wg[5], wg[7]
    
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
    Jacobian_C(x::Vector{<:Real}, wg::WristGeometry; intrinsic::Bool, split::Bool = false)

Differential kinematics of the constraints of comparative 2UPS + 1U mechanism.
"""
function Jacobian_2SPU1U(x::Vector{<:Real}, wg::Vector; intrinsic::Bool = true, split::Bool = false)

    q = inverse_kinematics_C(x, wg, intrinsic = intrinsic)

    # differentiation by autodiff
    Jx = ForwardDiff.jacobian(x_ -> constraints_C(x_, q, wg, intrinsic), x)
    Jq = ForwardDiff.jacobian(q_ -> constraints_C(x, q_, wg, intrinsic), q)

    if split
        return Jx, Jq
    else
        return -inv(Jx)*Jq
    end
end