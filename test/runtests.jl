using NovelWrist 
using Test

# testing forward and inverse solutions inside feasible region"
@testset "kinematic tests (intrinsic)" begin
    lim = RH5_wrist.actuator_limits
    for q1 in LinRange(lim[1][1], 0.145, 10)
        for q2 in LinRange(lim[2][1], 0.145, 10)
            x = NovelWrist.forward_kinematics([q1, q2], RH5_wrist, solution = [2,1,1])
            @test NovelWrist.inverse_kinematics(x, RH5_wrist, solution = [1,2]) ≈ [q1, q2]
        end
    end
end

@testset "kinematic tests (extrinsic)" begin
    lim = RH5_wrist.actuator_limits
    for q1 in LinRange(lim[1][1], 0.145, 10)
        for q2 in LinRange(lim[2][1], 0.145, 10)
            x = NovelWrist.forward_kinematics([q1, q2], RH5_wrist, solution = [2,1,1], intrinsic = false)
            @test NovelWrist.inverse_kinematics(x, RH5_wrist, solution = [1,2], intrinsic = false) ≈ [q1, q2]
        end
    end
end

