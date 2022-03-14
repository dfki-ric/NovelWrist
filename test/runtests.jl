using NovelWrist 
using Test

# testing forward and inverse solutions inside feasible region"
@testset "kinematic tests (intrinsic)" begin
    lim = RH5_wrist.actuator_limits
    for q1 in LinRange(lim[1][1], lim[1][2], 20)
        for q2 in LinRange(lim[2][1], lim[2][2], 20)
            x = NovelWrist.forward_kinematics([q1, q2], RH5_wrist, solution = [2,1,1])
            @test inverse_kinematics(x, RH5_wrist, solution = [1,2]) ≈ q
        end
    end
end

@testset "kinematic tests (extrinsic)" begin
    lim = RH5_wrist.actuator_limits
    for q1 in LinRange(lim[1][1], lim[1][2], 20)
        for q2 in LinRange(lim[2][1], lim[2][2], 20)
            x = NovelWrist.forward_kinematics([q1, q2], RH5_wrist, solution = [2,1,1], intrinsic = false)
            @test inverse_kinematics(x, RH5_wrist, solution = [1,2], intrinsic = false) ≈ q
        end
    end
end