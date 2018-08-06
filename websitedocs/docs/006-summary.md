---
title: Summary
---

![Under Construction](/img/worker-male-48.png) **Under Development:** The content of this page is still under development.

The following lists the API for the SimulationConstructionSet class. These methods are usually called when creating a simulation and are for setting the parameters of the simulation, such as the integration time step, the camera position and fix, and for specifying which variables are initially plotted in the graphs.  

There are two constructors for a SimulationConstructionSet. Both require a Robot, rob. In the second constructor, if boolean showGUI equals false, then no Graphical User Interface will be displayed. This is useful when you wish to run a simulation, or several simulations quickly, but do not need to interact through a GUI. In both cases, before the simulation can be simulated, you must first create a Thread, given the SimulationConstructionSet, and start the Thread.  

To start simulating when not using a GUI, use the simulate() methods. To know when the simulation is finished, use addSimulateDoneListener(). SimulateDoneListener is an interface that contains the single method simulateDone(). To stop the simulation, use stop().