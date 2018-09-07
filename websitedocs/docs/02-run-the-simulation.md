---
title: Run the simulation
---
 
Make sure that your editor does not complain about any compilation issues and run SimplePendulumSimulation. 

### 1. Select a variable to graph

As before, look for the "q_fulcrum" variable in the search tab of the Variable Viewer. Create a new graph and drag the "q_fulcrum" into it.  
Also, look for the "tau_fulcrum" variable and add it to a new graph. This latter variable represents the torque calculated by the controller to set the pendulum to its desired horizontal position.  

### 2. Simulate and Experiment

Click on the Simulate button: ![simulate](/img/scs-tutorials/scsSimulateButton.png) This time the pendulum should start moving and very rapidly stabilize toward its desired angle position value.  
Zoom in a little, notice that the graph accurately shows the trajectory of the pendulum at each time step. 

![pendulum initial state](/img/scs-tutorials/simple-pendulum/simple_pendulum_controller.gif)
 
You can now experiment with different initial parameters and controller gains.
  