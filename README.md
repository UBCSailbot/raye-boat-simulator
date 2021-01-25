# boat-simulator

This repository contains MATLAB scripts and Simulink models for the purpose of creating a low-level controller for UBC Sailbot. The model is based on  _Modeling and Nonlinear Heading Control of Sailing Yachts_ by Jerome Jouffroy. The pdf can be found [here](docs/Jouffroy-2013linjoe.pdf). 

# Quick Start Instructions

1. Install MATLAB and Simulink. This is free for UBC students, with instructions [here](https://it.ubc.ca/services/desktop-print-services/software-licensing/matlab). Note: for Linux, this is a bit more complicated. Contact Tyler Lum if you have any issues setting it up.

2. Clone the repository `git clone https://github.com/UBCSailbot/boat-simulator.git`

3. Open MATLAB and change your working directory to the `boat-simulator` folder that you just cloned

4. Run "initialize.m" 

# Variables

There are many variables in this complicated model. While the Jouffroy paper does a good job explaining the varibles, it is long and overwhelming to read and understand. Here, I will define the variables more clearly and highlight some easy misunderstandings (as of Aug 2019):

* Two notable coordiate frames:

    * n-frame - world frame with xyz = North-East-Down
    
    * b-frame - boat frame with xyz = Forward-Right-Down
    
* n = [x;y;phi;psi]

    * x - position in n-frame
    
    * y - position in n-frame
    
    * phi - angle around x-axis in n-frame
    
    * psi - angle around z-axis in n-frame
    
* vss = [surge;sway;roll;yaw]

    * surge - linear velocity forward in b-frame (u in paper)
    
    * sway - linear velocity right in b-frame (v in paper)
    
    * roll - angular velocity around x-axis in b-frame (p in paper)
    
    * yaw - angular velocity around z-axis in b-frame (r in paper)
    
* n_dot, v_dot - time derivatives of above variables

All of the following variables are vectors with 4 components, with the components directions matching vss above. This means component 1 is forward, 2 is right, 3 is angular around forward-axis, 4 is angular around down-axis.

* T - vector of propulsive forces (from rudder and sail)

* D - vector of damping forces

* g - vector of righting moments

All of the following are parameters that must be tuned/tested (not exhaustive list)

* (x_r, y_r, z_r) - Center of effort rudder

    * change r to s for sail, k for keel, h for hull
    
The following are functions that are complete approximations, might need some input from professors to see if valid

* lift = 0.5\*rho\*A\*(v^2)\*k1*\sin(2\*alpha)

* drag = 0.5\*rho\*A\*(v^2)\*k1\*(1-cos(2\*alpha))

* F_rh = v_ah^2