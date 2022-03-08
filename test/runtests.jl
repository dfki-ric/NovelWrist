using NovelWrist 
using Base.Test

println("testing kinematic function...")
t = @elapsed include("kinematic_test.jl")
println("done (took $t seconds).")
 