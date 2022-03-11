using NovelWrist 
using Test

println("testing kinematic functions...")
t = @elapsed include("kinematic_test.jl")
println("done (took $t seconds).")

println("testing plotting functions...")
t = @elapsed include("plotting_test.jl")
println("done (took $t seconds).")
