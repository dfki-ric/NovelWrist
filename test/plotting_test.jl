module plotting_test 
    using Test 
    using NovelWrist

    ###################
    # hardcoded tests #
    ###################

    build_wrist = WristGeometry(l = (0.045, 0.045), 
                                r = (0.049, 0.049), 
                                r_ = (0.049, 0.049),
                                h = (0.012, 0.012),
                                b = ([0.015, -0.178, -0.034], [-0.015, -0.178, -0.034]),
                                c = ([0.015, -0.032, 0.011], [-0.015, -0.032, 0.011]),
                                e0 = ([0.027, 0, -0.030], [-0.027, 0, -0.030]),
                                n = ([1, 0, 0], [-1, 0, 0]),
                                actuator_limits = ((0.113, 0.178), (0.113, 0.178))); 

    
    act1, act2 = plot_configuration_space(build_wrist, specsol = [1,2], resol = 3, testing = true) 
    tv1 = [NaN, 0.1893295387891028, NaN, NaN, 0.13347357815533836, NaN, NaN, 0.1893295387891028, NaN]
    for i in eachindex(act1)
        if isnan(act1[i]) && isnan(act2[i])
            @test 1 == 1  
        else 
            @test isapprox(act1[i], act2[i])
            @test isapprox(act1[i], tv1[i])
        end 
    end 
    

    wscond = plot_conditioning(build_wrist, α = (-π, π), specsol = [1,2], γ = (-π, π), resol = 3, testing = true)
    tv2 =  [NaN  0.8099999999999995  NaN
            NaN  0.8099999999999995  NaN
            NaN  0.8099999999999995  NaN]
    for i in 1:3
        for j in 1:3 
            if isnan(wscond[i,j]) && isnan(tv2[i,j])
                @test 1 == 1  
            else 
                @test isapprox(wscond[i,j], tv2[i,j])
            end
        end 
    end 

    wsdet = plot_singularities(build_wrist, α = (-π, π), specsol = [1,2], γ = (-π, π), resol = 3, testing = true)
    tv3 =  [NaN  -5.088883525189796  NaN
            NaN  -3.900179901275003  NaN
            NaN  -5.088883525189796  NaN]
    for i in 1:3
        for j in 1:3 
            if isnan(wsdet[i,j]) && isnan(tv3[i,j])
                @test 1 == 1  
            else 
                @test isapprox(wsdet[i,j], tv3[i,j])
            end
        end 
    end 
 

    wscond_c = plot_conditioning_C(build_wrist, α = (-π, π), γ = (-π, π), resol = 3, testing = true)
    tv4 = [ NaN   2.220446049250313e-16  NaN
            NaN  -8.881784197001252e-16  NaN
            NaN  -8.881784197001252e-16  NaN]
    for i in 1:3
        for j in 1:3 
            if isnan(wscond_c[i,j]) && isnan(tv4[i,j])
                @test 1 == 1  
            else 
                @test isapprox(wscond_c[i,j], tv4[i,j])
            end
        end 
    end 
         
    wsdet_2, wsdet_C_2 = plot_singularities_C(build_wrist, α = (-π, π), specsol = [1,2], γ = (-π, π), resol = 3, testing = true)    
    tv5 = [ NaN   -10.17776705037959  NaN
            NaN   -7.800359802550007  NaN
            NaN   -10.17776705037959  NaN] 
    tv6 = [ -7.375169979023237   -7.375169979023237  -7.375169979023237
            -7.375169979023237   -7.375169979023237  -7.375169979023237
            -7.375169979023237   -7.375169979023237  -7.375169979023237] 

    for i in 1:3
        for j in 1:3 
            if isnan(wsdet_2[i,j]) && isnan(tv5[i,j])
                @test 1 == 1  
            else 
                @test isapprox(wsdet_2[i,j], tv5[i,j])
            end
        end 
    end 

    for i in 1:3
        for j in 1:3 
            if isnan(wsdet_C_2[i,j]) && isnan(tv6[i,j])
                @test 1 == 1  
            else 
                @test isapprox(wsdet_C_2[i,j], tv6[i,j])
            end
        end 
    end 

    test_torques = plot_torque_C(build_wrist, α = (-π, π), γ = (-π, π), specsol = [1,2], resol=9, testing = true)
    tv7 = [53.2862308490202, 56.379471835827346, 44.97570047592406, 53.86416781404602]
 
    for i in 1:4
        @test isapprox(test_torques[i], tv7[i])
    end 
end 

