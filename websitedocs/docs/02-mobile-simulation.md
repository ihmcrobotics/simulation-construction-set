---
title: Mobile Simulation
---

### Create the class MobileSimulation
   Fill it in as follows:

<pre><code data-url-index="0" data-snippet="complete" id="MobileSimulation"></code></pre>

Note that the integration time step is set to 0.02 seconds: `sim.setDT(0.02, 1);`

This is a fairly large time step, but in this case it is possible since there are no high-frequency interactions in the system. In general, to set the integration time step, experiment with different values. As the time step gets higher, you will see different behavior due to numerical instabilities. Choose a time step which is low enough that making it any lower does not change the outcome of the simulation. 

The second parameter to setDT is 1, meaning that every integration step will be recorded.

Note that camera tracking and dollying are off and ground is set to invisible.

Instead of tracking and dollying the camera position and fix are set to get a good view of the mobile:

   `sim.setGroundVisible(false);`  
   `sim.setCameraTracking(false,false,false,false);`  
   `sim.setCameraDolly(false,false,false,false);`  
   `sim.setCameraPosition(1.0,1.0,0.5);`  
   `sim.setCameraFix(0.0,0.0,0.8);`  
   
<script id="snippetscript" src="../snippetautomation/codesnippets.js" sources=Array.of("https://rawgit.com/ihmcrobotics/ihmc-open-robotics-software/master/example-simulations/src/main/java/us/ihmc/exampleSimulations/mobile/MobileSimulation.java")></script>