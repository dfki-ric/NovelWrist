"""
    plot_configuration_space(wg::WristGeometry; solution::Vector{Int}, intrinsic::Bool = true, resol::Int = 100)

Scatter plot of the admissible configurations
"""
function plot_configuration_space(wg::WristGeometry; solution::Vector{Int}, intrinsic::Bool = true, resol::Int = 100)

    @assert length(solution) == 2 && all([s in Set([1,2]) for s in solution]) "Solution vector incorrect!"

    act1 = fill(NaN, (resol^2, ))
    act2 = fill(NaN, (resol^2, ))

    c = 1
    for (_, x) in enumerate(LinRange(-π, π, resol))
        for (_, y) in enumerate(LinRange(-π, π, resol))
            q = inverse_kinematics([x,y], wg, solution = solution, intrinsic = intrinsic)
            act1[c] = q[1]
            act2[c] = q[2]
            c += 1
        end
    end

    plt = scatter(act1, act2, aspect_ratio = :equal, size = (600,600), dpi = 300, xlabel = "q₁ [m]", ylabel = "q₂ [m]", label = false,  xlabelfontsize = 15, ylabelfontsize = 15, xtickfontsize=12, ytickfontsize=12)
    
    # plotting actuator limits
    plot!([wg.actuator_limits[1][1], wg.actuator_limits[1][1]], 
          [wg.actuator_limits[2][1], wg.actuator_limits[2][2]], color = :red, lw = 2, label = false)
    plot!([wg.actuator_limits[1][2], wg.actuator_limits[1][2]],
          [wg.actuator_limits[2][1], wg.actuator_limits[2][2]], color = :red, lw = 2, label = false)
    plot!([wg.actuator_limits[1][1], wg.actuator_limits[1][2]],
          [wg.actuator_limits[2][1], wg.actuator_limits[2][1]], color = :red, lw = 2, label = false)
    plot!([wg.actuator_limits[1][1], wg.actuator_limits[1][2]], 
          [wg.actuator_limits[2][2], wg.actuator_limits[2][2]], color = :red, lw = 2, label = false)

    
    return plt

end

"""
    plot_conditioning(wg::WristGeometry; α::Tuple{Real, Real}, γ::Tuple{Real, Real}, solution::Vector{Int}, intrinsic::Bool = true, resol::Int = 200)

Plots the conditioning for a predefined workspace
"""
function plot_conditioning(wg::WristGeometry; α::Tuple{Real, Real}, γ::Tuple{Real, Real}, solution::Vector{Int}, intrinsic::Bool = true, resol::Int = 200)

    @assert length(solution) == 2 && all([s in Set([1,2]) for s in solution]) "Solution vector incorrect!"

    xrange = LinRange(α[1], α[2], resol)
    yrange = LinRange(γ[1], γ[2], resol)
    wscond = fill(NaN, (resol, resol))
    shadow = fill(NaN, (resol, resol))

    wsareabounds = SVector.([2,1,-1,1,2], [0,-2,0,2,0])
    wsx_left,wsy_left,wsx_right,wsy_right = [],[],[],[]

    sprev = 0
    for (i, x) in enumerate(xrange)
        for (j, y) in enumerate(yrange)
            J = Jacobian([x,y], wg, solution = solution, intrinsic = intrinsic)
            q = inverse_kinematics([x,y], wg, solution = solution, intrinsic = intrinsic)

            if any(isnan.(J))
                wscond[i,j] = NaN
            else
                wscond[i,j] = 1/cond(Matrix{Real}(J*J'))
            end

            if all(vcat(q .< wg.actuator_limits[1][2], q .> wg.actuator_limits[1][1], inpolygon([x,y], wsareabounds; in=true, on=true, out=false)))
                shadow[i,j] = 1
            end

            if isone(shadow[i,j]) && any(isnan.(sprev))
                append!(wsx_left, x)
                append!(wsy_left, y)
            elseif isnan(shadow[i,j]) && any(isone.(sprev))
                append!(wsx_right, x)
                append!(wsy_right, y)
            end
            sprev = shadow[i,j]
        end
    end

    wsx = vcat(wsx_left, reverse!(wsx_right))
    wsy = vcat(wsy_left, reverse!(wsy_right))

    plt = heatmap(xrange, yrange, wscond', xlims = α, ylims = γ, aspect_ratio = :equal, size = (600,600), dpi = 300, xlabel = "α [rad]", ylabel = "γ [rad]", xlabelfontsize = 15, ylabelfontsize = 15, xtickfontsize=12, ytickfontsize=12)

    plot!(Shape(Real.(wsx), Real.(wsy)), fillcolor = plot_color(:blue2, 0.2), line = (1, :dash, :lightblue), label = "")

    return plt

end

"""
    plot_singularities(wg::WristGeometry; α::Tuple{Real, Real}, γ::Tuple{Real, Real}, solution::Vector{Int}, intrinsic::Bool = true, resol::Int = 200)

Plots singulartities for a predefined workspace
"""
function plot_singularities(wg::WristGeometry; α::Tuple{Real, Real}, γ::Tuple{Real, Real}, solution::Vector{Int}, intrinsic::Bool = true, resol::Int = 200)

    @assert length(solution) == 2 && all([s in Set([1,2]) for s in solution]) "Solution vector incorrect!"

    xrange = LinRange(α[1], α[2], resol)
    yrange = LinRange(γ[1], γ[2], resol)
    wsdet = fill(NaN, (resol, resol))
    shadow = fill(NaN, (resol, resol))

    wsareabounds = SVector.([2,1,-1,1,2], [0,-2,0,2,0])
    wsx_left,wsy_left,wsx_right,wsy_right = [],[],[],[]

    sprev = 0
    for (i, x) in enumerate(xrange)
        for (j, y) in enumerate(yrange)
            Jx, Jq = Jacobian([x,y], wg, solution = solution, intrinsic = intrinsic, split = true)
            q = inverse_kinematics([x,y], wg, solution = solution, intrinsic = intrinsic)

            if any(isnan.(Jx))
                wsdet[i,j] = NaN
            else
                wsdet[i,j] = log10(sqrt(det(Matrix{Real}(Jx*Jx'))))
            end

            if all(vcat(q .< wg.actuator_limits[1][2], q .> wg.actuator_limits[1][1], inpolygon([x,y], wsareabounds; in=true, on=true, out=false)))
                shadow[i,j] = 1
            end
            if isone(shadow[i,j]) && any(isnan.(sprev))
                append!(wsx_left, x)
                append!(wsy_left, y)
            elseif isnan(shadow[i,j]) && any(isone.(sprev))
                append!(wsx_right, x)
                append!(wsy_right, y)
            end
            sprev = shadow[i,j]
        end
    end

    wsx = vcat(wsx_left, reverse!(wsx_right))
    wsy = vcat(wsy_left, reverse!(wsy_right))

    plt = heatmap(xrange, yrange, wsdet', xlims = α, ylims = γ, size = (600,600), aspect_ratio = :equal, dpi = 300, xlabel = "α [rad]", ylabel = "γ [rad]", xlabelfontsize = 15, ylabelfontsize = 15, xtickfontsize=12, ytickfontsize=12)

    plot!(Shape(Real.(wsx), Real.(wsy)), fillcolor = plot_color(:blue2, 0.2), line = (1, :dash, :lightblue), label = "")

    return plt

end

"""
    plot_comparative_conditioning(wg::WristGeometry; α::Tuple{Real, Real}, γ::Tuple{Real, Real}, solution::Vector{Int} = [1,1], intrinsic::Bool = true, resol::Int = 200)

Plots the difference of the condition index (novel and conventional design) for a predefined workspace
"""
function plot_comparative_conditioning(wg::WristGeometry; α::Tuple{Real, Real}, γ::Tuple{Real, Real}, solution::Vector{Int}, intrinsic::Bool = true, resol::Int = 200)

    @assert length(solution) == 2 && all([s in Set([1,2]) for s in solution]) "Solution vector incorrect!"

    xrange = LinRange(α[1], α[2], resol)
    yrange = LinRange(γ[1], γ[2], resol)
    wscond = fill(NaN, (resol, resol))

    shadow = fill(NaN, (resol, resol))
    wsareabounds = SVector.([2,1,-1,1,2], [0,-2,0,2,0])
    wsx_left,wsy_left,wsx_right,wsy_right = [],[],[],[]

    sprev = 0
    for (i, x) in enumerate(xrange)
        for (j, y) in enumerate(yrange)
            J = Jacobian([x,y], wg, solution = solution, intrinsic = intrinsic)
            J_C = Jacobian_C([x,y], wg, intrinsic = intrinsic)
            q = inverse_kinematics([x,y], wg, solution = solution, intrinsic = intrinsic)

            if !any(isnan.(J))
               wscond[i,j] = 1/cond(Matrix{Real}(J*J')) - 1/cond(Matrix{Real}(J_C*J_C'))
            end
            if all(vcat(q .< wg.actuator_limits[1][2], q .> wg.actuator_limits[1][1], inpolygon([x,y], wsareabounds; in=true, on=true, out=false)))
                shadow[i,j] = 1
            end
            if isone(shadow[i,j]) && any(isnan.(sprev))
                append!(wsx_left, x)
                append!(wsy_left, y)
            elseif isnan(shadow[i,j]) && any(isone.(sprev))
                append!(wsx_right, x)
                append!(wsy_right, y)
            end
            sprev = shadow[i,j]
        end
    end
    wsx = vcat(wsx_left, reverse!(wsx_right))
    wsy = vcat(wsy_left, reverse!(wsy_right))

    my_pal = palette([:red, :white, :green], 90)

    plt = heatmap(xrange, yrange, wscond', xlims = α, ylims = γ, c = my_pal, aspect_ratio = :equal, size = (620,600), dpi = 300, xlabel = "α [rad]", ylabel = "γ [rad]", xlabelfontsize = 13, ylabelfontsize = 13, xtickfontsize=12, ytickfontsize=12, right_margin=7mm, bottom_margin = 0mm)
    plot!(Shape(Real.(wsx), Real.(wsy)), fillcolor = plot_color(:blue2, 0.0), line = (1, :dash, :black), label = "")

    return plt

end

"""
    plot_comparative_singularities(wg::WristGeometry; α::Tuple{Real, Real}, γ::Tuple{Real, Real}, solution::Vector{Int}, intrinsic::Bool = true, resol::Int = 5000)

Plots the singularity curves (novel and conventional design)
"""
function plot_comparative_singularities(wg::WristGeometry; α::Tuple{Real, Real}, γ::Tuple{Real, Real}, solution::Vector{Int}, intrinsic::Bool = true, resol::Int = 700)

    @assert length(solution) == 2 && all([s in Set([1,2]) for s in solution]) "Solution vector incorrect!"

    xrange = LinRange(α[1], α[2], resol)
    yrange = LinRange(γ[1], γ[2], resol)
    wsdet = fill(NaN, (resol, resol))
    wsdet_C = fill(NaN, (resol, resol))

    shadow = fill(NaN, (resol, resol))
    wsareabounds = SVector.([2,1,-1,1,2], [0,-2,0,2,0])
    wsx_left,wsy_left,wsx_right,wsy_right = [],[],[],[]

    sprev = 0
    for (i, x) in enumerate(xrange)
        for (j, y) in enumerate(yrange)
            Jx, _ = Jacobian([x,y], wg, solution = solution, intrinsic = intrinsic, split = true)
            Jx_C, _ = Jacobian_C([x,y], wg, intrinsic = intrinsic, split = true)

            q = inverse_kinematics([x,y], wg, solution = solution, intrinsic = intrinsic)
            
            if any(isnan.(Jx))
                wsdet[i,j] = NaN
            else
                wsdet[i,j] = log10(det(Matrix{Real}(Jx*Jx')))
            end
            wsdet_C[i,j] = log10(det(Matrix{Real}(Jx_C*Jx_C')))

            if all(vcat(q .< wg.actuator_limits[1][2], q .> wg.actuator_limits[1][1], inpolygon([x,y], wsareabounds; in=true, on=true, out=false)))
                shadow[i,j] = 1
            end
            if isone(shadow[i,j]) && any(isnan.(sprev))
                append!(wsx_left, x)
                append!(wsy_left, y)
            elseif isnan(shadow[i,j]) && any(isone.(sprev))
                append!(wsx_right, x)
                append!(wsy_right, y)
            end
            sprev = shadow[i,j]
        end
    end

    wsx = vcat(wsx_left, reverse!(wsx_right))
    wsy = vcat(wsy_left, reverse!(wsy_right))

    shadow = deepcopy(wsdet)
    for i = 1:resol
        for j = 1:resol
            if !isnan(shadow[i,j])
                shadow[i,j] = 1
            end
        end
    end

    plt= heatmap(xrange, yrange, shadow', xlims = α, ylims = γ, size = (600,600), dpi = 500, xlabel = "α [rad]", ylabel = "γ [rad]", xlabelfontsize = 15, ylabelfontsize = 15, xtickfontsize=12, ytickfontsize=12, colorbar = :false, aspect_ratio = :equal, color = RGBA(111/256, 166/256, 230/256, 64/256))

    contour!(xrange, yrange, wsdet', levels = [-13.5], color = :green)
    contour!(xrange, yrange, wsdet_C', levels = [-13.5], color = :red)

    plot!([0, 0], [0, 0], color = :green, label = "new wrist", legend = :bottom)

    plot!([0, 0], [0, 0], color = :red, label = "comp. design")

    # plotting shape of feasible work space
    plot!(Shape(Real.(wsx), Real.(wsy)), fillcolor = plot_color(:blue2, 0.0), line = (1, :dash, :black), label = "")

    return plt

end 

"""
    plot_comparative_torque(wg::WristGeometry; α::Tuple{Real, Real}, γ::Tuple{Real, Real}, solution::Vector{Int}, intrinsic::Bool = true, resol::Int = 300)

Comparative plot for delivered pure inclination/ pitch torque and speed
"""
function plot_comparative_torque(wg::WristGeometry; α::Tuple{Real, Real}, γ::Tuple{Real, Real}, solution::Vector{Int}, intrinsic::Bool = true, resol::Int = 300)

    xrange = LinRange(α[1], α[2], resol)
    yrange = LinRange(γ[1], γ[2], resol)
    f1 = fill(NaN, (resol,))
    f2 = fill(NaN, (resol,))
    f1_C = fill(NaN, (resol,))
    f2_C = fill(NaN, (resol,))
    s1 = fill(NaN, (resol,))
    s2 = fill(NaN, (resol,))
    s1_C = fill(NaN, (resol,))
    s2_C = fill(NaN, (resol,))

    # determine the limits of the optimized workspace
    wsareabounds = SVector.([2,1,-1,1,2], [0,-2,0,2,0])
    wsx,wsy = [],[]
    shadow = fill(NaN, (resol, resol))
    sprev = 0

    for (i, x) in enumerate(xrange)
       for (j, y) in enumerate(yrange)
            q = inverse_kinematics([x,y], wg, solution = solution, intrinsic = intrinsic)
            if all(vcat(q .< wg.actuator_limits[1][2], q .> wg.actuator_limits[1][1], inpolygon([x,y], wsareabounds; in=true, on=true, out=false)))
                shadow[i,j] = 1
            end

            if isone(shadow[i,j]) && any(isnan.(sprev))
                append!(wsx, x)
                # for pure tilt movement, only consider values where inclination is zero
                if isapprox(x, 0, atol = abs(α[2]-α[1])/resol)
                    append!(wsy, y)
                end
            elseif isnan(shadow[i,j]) && any(isone.(sprev))
                append!(wsx, x)
                # for pure tilt movement, only consider values where inclination is zero
                if isapprox(x, 0, atol = abs(α[2]-α[1])/resol)
                    append!(wsy, y)
                end
            end
            sprev = shadow[i,j]
       end
    end
    αlims = (minimum(wsx),maximum(wsx))
    γlims = (minimum(wsy),maximum(wsy))

    αlim_comp = 0 # holds α value for singularity that occurs at pure inclination of comparative design
    f1_C_prev = 0 # holds inclination torque value of comparative design for previous timestep 

    for (i, x) in enumerate(xrange)
        if x > αlims[1] && x < αlims[2]
            J_C = Jacobian_C([x,0], wg, intrinsic = intrinsic)
            J = Jacobian([x,0], wg, solution = solution, intrinsic = intrinsic)
            if any(isnan.(J))
                f1[i] = NaN
                s1[i] = NaN
            else
                f1[i] = (inv(transpose(J))*[1000, 1000])[1] # N - actuation space
                s1[i] = (J*[0.2, 0.2])[1]                   # mm/s - actuation space
            end
            if any(isnan.(J_C))
                f1_C[i] = NaN
                s1_C[i] = NaN
            else
                f1_C[i] = (inv(transpose(J_C))*[1000, 1000])[1]
                s1_C[i] = (J_C*[0.2, 0.2])[1]
            end
        else
            f1[i] = NaN
            s1[i] = NaN
            f1_C[i] = NaN
            s1_C[i] = NaN
        end
        # check at which step the inclination torque of the comp. design passes the zero axis to find the workspace singularity 
        if f1_C_prev > 0 && f1_C[i] < 0
            αlim_comp = xrange[i-1]
        end      
        f1_C_prev = f1_C[i]
    end

    for (j, y) in enumerate(yrange)
        if y > γlims[1] && y < γlims[2]
            J_C = Jacobian_C([0,y], wg, intrinsic = intrinsic)
            J = Jacobian([0,y], wg, solution = solution, intrinsic = intrinsic)
            if any(isnan.(J))
                f2[j] = NaN
                s2[j] = NaN
            else
                f2[j] = (inv(transpose(J))*[1000, -1000])[2]
                s2[j] = (J*[0.2, -0.2])[2]
            end
            if any(isnan.(J_C))
                f2_C[j] = NaN
                s2_C[j] = NaN
            else
                f2_C[j] = (inv(transpose(J_C))*[1000, -1000])[2]
                s2_C[j] = (J_C*[0.2, -0.2])[2]
            end
        else
            f2[j] = NaN
            s2[j] = NaN
            f2_C[j] = NaN
            s2_C[j] = NaN
        end
    end


    # display characteristic values for pure inclination/ pure tilt movements

    # maximum torques 
    maxincltorque_nw, maxinlcidx_nw = nonNaNmax(f1) # maximum inclination torque - new wrist  
    maxtilttorque_nw, maxtiltidx_nw = nonNaNmax(f2) # maximum tilt torque - new wrist 
    maxincltorque_cd, maxinlcidx_cd = nonNaNmax(f1_C) # maximum inclination torque - comparative design  
    maxtilttorque_cd, maxtiltidx_cd = nonNaNmax(f2_C) # maximum tilt torque - comparative design 

    # correspondant velocities 
    inclvelocity_nw = s1[maxinlcidx_nw] # inclination velocity at maximum inclination torque - new wrist
    tiltvelocity_nw = s2[maxtiltidx_nw] # tilt velocity at maximum tilt torque - new wrist
    inclvelocity_cd = s1_C[maxinlcidx_cd] # inclination velocity at maximum inclination torque - comp design
    tiltvelocity_cd = s2_C[maxtiltidx_cd] # tilt velocity at maximum tilt torque - comp design
    
    if !testing
        # print characteristics to console
        str1 = "Inclination range: $(round(αlims[1], digits = 2))/$(round(αlims[2], digits = 2)) rad, \nMaximum inclination torque: $(round(maxincltorque_nw, digits = 2)) Nm, correspondent inclination velocity: $(round(inclvelocity_nw, digits = 2)) rad/s, \nTilt range: $(round(γlims[1], digits = 2))/$(round(γlims[2], digits = 2)) rad, \nMaximum tilt torque: $(round(maxtilttorque_nw, digits = 2)) Nm, correspondent tilt velocity: $(round(tiltvelocity_nw, digits = 2)) rad/s"
        str2 = "Inclination range: $(round(αlims[1], digits = 2))/$(round(αlim_comp, digits = 2)) rad, \nMaximum inclination torque: $(round(maxincltorque_cd, digits = 2)) Nm, correspondent inclination velocity: $(round(inclvelocity_cd, digits = 2)) rad/s, \nTilt range: $(round(γlims[1], digits = 2))/$(round(γlims[2], digits = 2)) rad, \nMaximum tilt torque: $(round(maxtilttorque_cd, digits = 2)) Nm, correspondent tilt velocity: $(round(tiltvelocity_cd, digits = 2)) rad/s"

        println("Pure inclination/tilt characteristics - new wrist:\n"*str1*"\n")
        println("Pure inclination/tilt characteristics - comparative design:\n"*str2)
    end 
    ## force transmission
    # choose x-axis limits according to limits of the optimized workspace
    xlimits = (minimum([αlims[1],γlims[1]]), maximum([αlims[2],γlims[2]]))
    plt = plot(xrange, f1,
               xlims = xlimits,
               lw = 1,
               ls = :solid,
               color = :green,
               label = "inclination torque - new wrist ",
               xlabel = "α, γ [rad]", ylabel = "f₁, f₂ [Nm]",
               size = (600,400),
               dpi = 300,
               legend = :bottomleft,
               rightmargin = 12Plots.mm)

    plot!(xrange, f1_C,
            lw = 1,
            ls = :solid,
            color = :red,
            label = "inclination torque - comp. design")
     

    plot!(yrange, f2,
          xlims = xlimits,
          lw = 1,
          ls = :dash,
          color = :green,
          label = "tilt torque - new wrist ")


    plot!(yrange, f2_C,
          lw = 1,
          ls = :dash,
          color = :red,
          label = "tilt torque - comp. design")

    # plot ghost items to force grouped labels
    plot!([0], [0],
        lw = 1,
        ls = :dot,
        color = :green,
        label = "inclination speed - new wrist")
    plot!([0], [0],
            lw = 1,
            ls = :dot,
            color = :red,
            label = "inclination speed - comp. design")
    plot!([0], [0],
            lw = 1,
            ls = :dashdot,
            color = :green,
            label = "tilt speed - new wrist")
    plot!([0], [0],
        lw = 1,
        ls = :dashdot,
        color = :red,
        label = "tilt speed - comp. design")

    ## velocity transmission
    # find axis limits for second y-axis 
    ylimits = (minimum([nonNaNmin(s1)[1],nonNaNmin(s2)[1]])-8, maximum([nonNaNmax(s1)[1],nonNaNmax(s2)[1]]))
    plot!(twinx(plt), xrange, s1,
        xlims = xlimits,
        ylims = ylimits,
        lw = 1,
        ls = :dot,
        color = :green,
        label = "",
        ylabel = "dα/dt, dγ/dt [rad/s]") 

    plot!(twinx(plt), xrange, s1_C,
        xlims = xlimits,
        ylims = ylimits,
        lw = 1,
        ls = :dot,
        color = :red,
        label = "")
        
    plot!(twinx(plt),yrange, s2,
        xlims = xlimits,
        ylims = ylimits,
        lw = 1,
        ls = :dashdot,
        color = :green,
        label = "")

    plot!(twinx(plt), xrange, s2_C,
        xlims = xlimits,
        ylims = ylimits,
        lw = 1,
        ls = :dashdot,
        color = :red,
        label = "")
        
    return plt
  
end