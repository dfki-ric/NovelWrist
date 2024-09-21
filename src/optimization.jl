using NovelWrist

begin
    # defining optimization model
    model = Model(Ipopt.Optimizer)
    set_silent(model)
    # taking start values from actual design
    @variable(model, l[1:2])
    @variable(model, r[1:2])
    @variable(model, r_[1:2])
    @variable(model, h[1:2])
    @variable(model, b1[1:3])
    @variable(model, b2[1:3])
    @variable(model, c1[1:3])
    @variable(model, c2[1:3])
    @variable(model, e01[1:3])
    @variable(model, e02[1:3])
    @variable(model, n1[1:3])
    @variable(model, n2[1:3])

    set_start_value.(l, [0.045, 0.045])
    set_start_value.(r, [0.049, 0.049])
    set_start_value.(r_, [0.049, 0.049])
    set_start_value.(h, [0.012, 0.012])
    set_start_value.(b1, [0.015, -0.178, -0.034])
    set_start_value.(b2, [-0.015, -0.178, -0.034])
    set_start_value.(c1, [0.015, -0.032, 0.011])
    set_start_value.(c2, [-0.015, -0.032, 0.011])
    set_start_value.(e01, [0.027, 0, -0.030])
    set_start_value.(e02, [-0.027, 0, -0.030])
    set_start_value.(n1, [1, 0, 0])
    set_start_value.(n2, [-1, 0, 0])

    # for symmetric desings
    @constraint(model, l[1] == l[2])
    @constraint(model, r[1] == r[2])
    @constraint(model, r_[1] == r_[2])
    @constraint(model, h[1] == h[2])
    @constraint(model, b1 == [-b2[1], b2[2], b2[3]])
    @constraint(model, c1 == [-c2[1], c2[2], c2[3]])
    @constraint(model, e01 == [-e02[1], e02[2], e02[3]])
    @constraint(model, n1 == [-n2[1], n2[2], n2[3]])

    # setting constraints on the variables as bounds
    @constraint(model, l[1] <= 0.1)
    @constraint(model, l[1] >= 0.)
    @constraint(model, r[1] <= 0.1)
    @constraint(model, r[1] >= 0.)
    @constraint(model, r_[1] <= 0.1)
    @constraint(model, r_[1] >= 0.)
    @constraint(model, h[1] <= 0.1)
    @constraint(model, h[1] >= 0.)

    @objective(model, Max, workspace_conditioning([l, r, r_, h, [b1, b2], [c1, c2], [e01, e02], [n1, n2]], solution = [1,2]))
end

optimize!(model)
value.()

