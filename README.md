# boat-simulator

This repository contains MATLAB scripts and Simulink models for the purpose of creating an MPC controller for UBC Sailbot. As of Aug 2019, all the equations are based off of _Modeling and Nonlinear Heading Control of Sailing Yachts_ by Jerome Jouffroy. The pdf can be found [here](Jouffroy-2013linjoe.pdf). 

# Overview of the Model

To build an MPC controller we need to first build a boat mathematical model that accurately captures the dynamics of the boat. This is the most important, challenging, and time-consuming part of the controller. From there, we can make a simple controller that uses the model to choose appropriate actions to navigate to where we want the boat to go.

The model input variables that we control are the rudder and sail angle of the boat. The model input variables that we don't control are the wind speed and direction. The state variables that we care about are the boats x-y position and roll-yaw angle, as well as their derivatives. We ignore the z position and pitch angle in this model.

# Description

As of Aug 2019, we have had the following design changes.

* We started off by creating a model of the boat using the Jouffroy paper using the MATLAB Symbolic Toolbox in `nonlinear_sailboat_motion.m` (written by Nick). The goal was to use it to linearize the model for the controller. This was a great starting point, but the Symbolic Toolbox had issues with finding the x-y components of zero-vectors. This led to division by zeros that broke the model. In addition, the model was unstable and difficult to debug using the Symbolic Toolbox. 

* Next, we created `symless_nonlinear.m` to recreate what was done in `nonlinear_sailboat_motion.m`, but without the Symbolic Toolbox (Bruce and Tyler). This was in hopes of solving the division by zero problem and then debugging from there. This solution still encountered many issues and was not easy to debug, visualize or improve.

* Next, we created `sailbot_library.slx`, which contains Simulink blocks that match the equations of `symless_nonlinear.m`. Then we assembled the blocks in `Simulink_models/Assemble_Blocks_Aug_24.slx`, and then began debugging the system. So far, the division by zero issue seems to be gone, but the boat is still unstable. We are hoping to figure out the cause of the problems and then work from there (eg. sign error, parameter estimate issues, lift/drag/F_rh approximation errors, etc.)

## Old Message in README

Please go to the following link to view the repo in the google drive (temporary fix)
https://drive.google.com/drive/folders/1FMQCa-9aBP0aNRxnLEseIqyThMgN8272
