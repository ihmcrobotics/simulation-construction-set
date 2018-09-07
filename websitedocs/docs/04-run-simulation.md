---
title: Run the simulation
---

Make sure that your editor does not complain about any compilation issues and run SimplePendulumSimulation. 

### 1. Select a variable to graph

In the Variable Search field press the space bar. This will show all ten variables in the SCS system. 
Look for the "q_FulcrumPin" variable in the search tab of the Variable Viewer, create a new graph and drag the "q_FulcrumPin" into it.  
This variable represents the value of the pivot angle in radians and should oscillate between -π/2 and π/2.



### 2. Simulate

Click on the Simulate button: ![simulate](/img/scs-tutorials/scsSimulateButton.png) The pendulum should start swinging very rapidly and finally stop after converging to its equilibrium position.  
The simulation should stop shortly after the pendulum position has converged. That's perfectly normal.
If you remember we had set the simulation time parameter to 60 seconds and in simulate mode, SCS will run as fast as possible, in this case, faster than real-time.  

![pendulum initial state](/img/scs-tutorials/simple-pendulum/pendulum-after-run-sim.png)  
*Initial state of the pendulum - Starting from an angle of 90 degrees (q_Pinjoint value of 1.57 or π/2), converges to 0 degrees after few seconds:*

Zoom in a little while sliding the graph to the left, notice that the graph accurately shows the oscillations of the pendulum at each time step.   

### 3. Replay

Click and drag your mouse cursor on the graph to position the current time stamp to the beginning of the simulation (a black vertical bar should follow your movement as you drag the mouse on the graph and the pendulum should also move).  
or 
Press the Goto In Point button ![gotoIn](/img/scs-tutorials/scsGotoInPointButton.png) to rewind to the start of the simulation.

If you press the Play button ![play](/img/scs-tutorials/scsReplayButton.png) the simulation will playback in real-time.
Click on the play button, now the pendulum will swing back and forth and stop after 60 seconds. Interestingly you will notice that if you run the same simulation code twice in a row, it will produce the exact same trajectories due to starting from exactly the same initial conditions.  
If you were to start from slightly different conditions, it should produce different behavior.

![pendulum initial state](/img/scs-tutorials/simple-pendulum/no_control_simple_pendulum.gif)


### 4. Experiment

You can now experiment with different initial parameters and damping. 
For example first reset the sim to time 0, change a value such as "b_damp_FulcrumPinJoint" and press the Simulate button again to see how it affects the convergence time of the pendulum.

Additionally in the robot class experiment with different lengths, damping values, masses, and center of mass locations until you are comfortable with this simulation.

In the next tutorial we are going to add some simple control to this pendulum. 


  
  
  
  
  

