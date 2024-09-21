module NovelWrist 

# imported packages
using LinearAlgebra
using Plots
using ForwardDiff
using UnPack
using StaticArrays
using PolygonOps
using Oscar;   

using JuMP
using Ipopt

# Includes 
include("utils.jl")
include("kinematics.jl")
include("plotting.jl")

# kinematics
export inverse_kinematics, Jacobian

# performance analysis
export plot_configuration_space, plot_conditioning, plot_conditioning_C,plot_singularities, plot_singularities_C, plot_torque_C

end 