module kinematic_test
    using Test 
    using NovelWrist

    build_wrist = WristGeometry(l = (0.045, 0.045), 
                                r = (0.049, 0.049), 
                                r_ = (0.049, 0.049),
                                h = (0.012, 0.012),
                                b = ([0.015, -0.178, -0.034], [-0.015, -0.178, -0.034]),
                                c = ([0.015, -0.032, 0.011], [-0.015, -0.032, 0.011]),
                                e0 = ([0.027, 0, -0.030], [-0.027, 0, -0.030]),
                                n = ([1, 0, 0], [-1, 0, 0]),
                                actuator_limits = ((0.113, 0.178), (0.113, 0.178))); 
    
    @test inverse_kinematics([0,0], build_wrist, specsol = [1,2], intrinsic = true) ≈ [0.13347357815533836, 0.13347357815533836]
    @test Jacobian(x, build_wrist; specsol = [1,2], intrinsic = true, split = false)[1,1] ≈ 15.963257027677201
    @test Jacobian(x, build_wrist; specsol = [1,2], intrinsic = true, split = false)[1,2] ≈ 15.963257027677201
    @test Jacobian(x, build_wrist; specsol = [1,2], intrinsic = true, split = false)[2,1] ≈ 17.736952252974675
    @test Jacobian(x, build_wrist; specsol = [1,2], intrinsic = true, split = false)[2,2] ≈ -17.736952252974675

end 

 