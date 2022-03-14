# NovelWrist.jl
A Julia package for the kinematic analysis of the $`2S\underbar{P}U+2RSU+1U`$ mechanism that is used as wrist mechanism for the humanoid robot RH5v2, developed at the [DFKI Robotics Innovation Center](https://robotik.dfki-bremen.de/en/startpage.html). 

## Introduction
![test](./assets/humanoid_wrist_plane.png?raw=true "CAD design of $`2S\underbar{P}U+2RSU+1U`$ mechanism")

The here presented 2-DOF mechanism for inclination and tilt possesses two double closed-loop chains and allows increased range of motion compared to a classical 

## Installation
```jl
pkg> add NovelWrist
```

## Documentation
### Create a new design 
![test](./assets/kinematic_model.png?raw=true "kinematic model")

A wrist geometry is defined by the constructor taking keyword arguments related to the geometry above. The wrist mechanism at DFKI Bremen has the following geometry:

```jl
julia> using NovelWrist

julia> RH5_wrist = WristGeometry(l = (0.045, 0.045), 
                    	         r = (0.049, 0.049), 
                          	 r_ = (0.049, 0.049),
                          	 h = (0.012, 0.012),
                        	 b = ([0.015, -0.178, -0.034], [-0.015, -0.178, -0.034]),
                          	 c = ([0.015, -0.032, 0.011], [-0.015, -0.032, 0.011]),
                          	 e0 = ([0.027, 0, -0.030], [-0.027, 0, -0.030]),
                          	 n = ([1, 0, 0], [1, 0, 0]),
                          	 actuator_limits = ((0.113, 0.178), (0.113, 0.178))); 
```

The actuator limits denote the minimum and maximum values that can be reached by the linear actuators, denoted as 'q' in the kinematic model. Presented function calls are executed for the assembly mode of 'RH5_wrist' ('solution = [1,1]') but can be altered.

### Kinematics
#### Inverse Kinematics 
Computation of the actuator length from a given pose defined by *inclination* ($`\alpha`$) and *tilt* ($`\gamma`$). Note that 'solution' defines the which intersection points to pick from both sides of the circle-sphere intersectio. 

```jl
julia> x = [0, 0]; # angles in rad  

julia> q = inverse_kinematics(x, build_wrist; solution = [1,1], intrinsic = true)
2-element Vector{Real}:
 0.13347357815533836
 0.13347357815533836
```

#### Forward Kinematics
Computes the end-effector orientation `α` and `γ`, given the solution for the actuator lengths 'q':

```jl
julia> α, γ = forward_kinematics(q, build_wrist, specsol = [1,1]) 
2-element Vector{Real}:
 0.00 # TO BE TESTED... 
 0.00
```



#### Constrained Jacobian
To get the Jacobian **J** as product of the inverted work space Jacobian **J**x and the joint space Jacobian **J**q:

```jl
julia> J = Jacobian(x, build_wrist; specsol = [1,1], intrinsic = true, split = false)
2×2 Matrix{Real}:
 15.9633   15.9633
 17.737   -17.737
```
When `split = true`, **J**x and **J**q are returned componentwise. 

### Performance Analysis
#### Conditioning
The condition index of the novel mechanism can be plotted over `α` and `γ`:

```jl
julia> plot_conditioning(build_wrist, α = (-π, π), γ = (-π, π), solution = [1,1], resol = 500) # increasing resol will give a higher resolution
```
![test](./assets/condition_index.png?raw=true "Conditioning")
The dashed lines indicate the workspace limits imposed by `actuator_limits`.

#### Configuration Space
The actuator lengths for plotting the the configuration space are computed for end-effector orientations between -π and π: 
```jl
julia> plot_configuration_space(build_wrist; solution = [1,1], intrinsic = true, resol = 100)
```
![test](./assets/c_space.png?raw=true "Configuration space")
Here, for better visibility, the `actuator_limits` are visualized using a red rectangle. 

#### Comparison to Conventional Wrist Designs

Computes and plots the **difference of the condition index** between $`2S\underbar{P}U+2RSU+1U`$ and $`2S\underbar{P}U+1U`$ mechanism (positive values indicate increased dexterity of the novel design): 

```jl
julia> plot_comparative_conditioning(build_wrist, α = (-π, π), γ = (-π, π), solution = [1,1], resol = 400)
```
![test](./assets/conditioning_comparison.png?raw=true "Comparison of conditioning")


The **singularity curves** of novel design and comparative design are obtained by sampling through the work space. Note, that in order to get closed contures, a high value for `resol` has to be set. This however increases the computing time considerably.        

```jl
julia> plot_comparative_singularities(build_wrist, α = (-π, π), γ = (-π, π), solution = [1,1], intrinsic = true, resol = 5000)
```
![test](./assets/singularities_C.png?raw=true "Comparison of singularity curves")
The theoretically feasible work space for the novel design is denoted by the blue coloured "shadow".

Plots of **Torque** and **Speed** at pure inclination and pure tilt movements can be computed. Additionally, characteristic values are printed to the console:

```jl
julia> plot_torque_C(build_wrist, α = (-π, π), γ = (-π, π), solution = [1,1], resol=600)
    Pure inclination/tilt characteristics - new wrist:
    Inclination range: -0.74/1.83 rad, 
    Maximum inclination torque: 62.94 Nm, correspondent inclination velocity: 6.36 rad/s, 
    Tilt range: -0.97/0.98 rad, 
    Maximum tilt torque: 56.38 Nm, correspondent tilt velocity: 7.09 rad/s

    Pure inclination/tilt characteristics - comparative design:
    Inclination range: -0.74/1.76 rad, 
    Maximum inclination torque: 59.86 Nm, correspondent inclination velocity: 6.68 rad/s, 
    Tilt range: -0.97/0.98 rad, 
    Maximum tilt torque: 53.86 Nm, correspondent tilt velocity: 7.43 rad/s
```
![test](./assets/torque_and_speed.png?raw=true "Comparison of torque and speed at pure inclination/ tilt")


