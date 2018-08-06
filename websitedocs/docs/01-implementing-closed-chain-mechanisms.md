---
title: Implementing Closed-Chain Mechanisms
---

Currently, the Simulation Construction Set only allows for systems which have a tree structure of Joints and Links. 
However, Closed-Chain Mechanisms, like 4 bar linkages for example, can be implemented using ExternalForcePoints. 
Two ExternalForcePoints can be placed at two different points on the system which are constrained to stay together.
A "glue" force can then be computed based on the error in position between the two points. That glue force can then be applied to the ExternalForcePoints in equal and opposite directions. 
In this tutorial, you'll see how to construct a Flyball Governor simulation, like the one shown in Figure 8. 
Since this is a closed-loop mechanism, we use ExternalForcePoints to hold the cross links to the rotating blue cylinder. 

![Figure 8](/img/documentation/Figure8Flyball.PNG)

### Create a new package called flyballGovernor 
   The package should be located within your us.ihmc.exampleSimulations package.