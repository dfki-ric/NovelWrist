# initialize wrist design 
build_wrist = WristGeometry(l = (0.045, 0.045), 
                            r = (0.049, 0.049), 
                            r_ = (0.049, 0.049),
                            h = (0.012, 0.012),
                            b = ([0.015, -0.178, -0.034], [-0.015, -0.178, -0.034]),
                            c = ([0.015, -0.032, 0.011], [-0.015, -0.032, 0.011]),
                            e0 = ([0.027, 0, -0.030], [-0.027, 0, -0.030]),
                            n = ([1, 0, 0], [-1, 0, 0]),
                            actuator_limits = ((0.113, 0.178), (0.113, 0.178)))

# make use of the plot functions that include the kinematics
plot_conditioning(build_wrist, α = (-π, π), specsol = [1,2], γ = (-π, π), resol = 500)
savefig(String(@__DIR__)*"/conditioning.png")

plot_singularities(build_wrist, α = (-π, π), specsol = [1,2], γ = (-π, π), resol = 500)
savefig(String(@__DIR__)*"/singularities.png")

plot_conditioning_C(build_wrist, α = (-π, π), γ = (-π, π), resol = 400)
savefig(String(@__DIR__)*"/conditioning_C.png")

create_singularity_data(build_wrist, α = (-π, π), γ = (-π, π), resol = 5000)
plot_singularities_C()
savefig(String(@__DIR__)*"/singularities_C.png")

plot_torque_C(build_wrist, α = (-π, π), γ = (-π, π), specsol = [1,2], resol=600)
savefig(String(@__DIR__)*"/torque_C.png")
