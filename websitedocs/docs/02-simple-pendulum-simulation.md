---
title: SimplePendulumSimulation.java
---

Every SCS simulation requires an instance of `us.ihmc.simulationconstructionset.SimulationConstructionSet` and a Thread to run it.   IHMC's convention is to create a 'simulation' class which performs this function.
 
### 1. Create the SimplePendulumSimulation Class

In the java directory of your project, `src/main/java`, create a new java class `us.ihmc.exampleSimulations.simplePendulum.SimplePendulumSimulation`.
The simulation class is responsible for instantiating an instance of `SimulationConstructionSet` and starting a new `Thread` to run it.

The simplest version of this is as follows:  

```java
package us.ihmc.exampleSimulations.simplePendulum;

import us.ihmc.simulationconstructionset.SimulationConstructionSet;

/**
 * SimplePendulumSimulation
 */
public class SimplePendulumSimulation
{
   private SimulationConstructionSet sim;

   public SimplePendulumSimulation()
   {
      sim = new SimulationConstructionSet();

      Thread myThread = new Thread(sim);
      myThread.start();
   }

   public static void main(String[] args)
   {
      new SimplePendulumSimulation();
   }
}
```

* **Copy the Code and Run the Simulation**
Replace your class contents with the above and run the simulation.
Don't forget to add `-Xms4096m -Xmx4096m` to your VM's run configuration as specified in the [Quick Start](https://ihmcrobotics.github.io/ihmc-open-robotics-software/docs/quickstarthome.html). Since there is no robot in your simulation, you will just get an empty world as follows:
    ![blank simulation](/img/documentation/scsTutorial/blank-Simulation.png)

### 2.Set the Simulation Parameters

Before we add a robot, there are some basic parameters and settings that you should apply to `SimulationConstructionSet`.  Replace the simulation class contents with the following:


<pre><code data-url-index="0" data-snippet="complete" id="SimplePendulum"></code></pre>


<script id="snippetscript" src="../snippetautomation/codesnippets.js" sources=Array.of("https://rawgit.com/ihmcrobotics/ihmc-open-robotics-software/master/example-simulations/src/main/java/us/ihmc/exampleSimulations/simplePendulum/SimplePendulumSimulation.java")></script>

<br>

If you run the simulation now, it will not look any different, but what you have done is:

* **Create Simulation Parameters**  
- `SimulationConstructionSetParameters parameters = new SimulationConstructionSetParameters();`
- Instantiate a `SimulationConstructionSetParameters` object
- `parameters.setDataBufferSize(32000);` sets the initial data buffer size to be 32000 bytes.

* **Create a new Simulation and set its properties**
- `sim = new SimulationConstructionSet(parameters);`  
- Instantiate a new Runnable SimulationConstructionSet object with default values and a buffer size of 32000 bytes as defined in `parameters` object   
- `sim.setDT(DT, 20);`  sets the simulation's delta time value to be 20 milliseconds.             
- `sim.setGroundVisible(true);`  sets the ground to be visible in the 3D view.  
- `sim.setCameraPosition(0, -9.0, 0.6);`  
  `sim.setCameraFix(0.0, 0.0, 0.70);`  sets the location and orientation of the camera in the 3D world.  
- `sim.setSimulateDuration(60.0);`  specifies that the simulation will only run for a duration of 60 seconds.  For this tutorial, this allows the simulation to run to a point where it does not overflow the data buffer.

Now it's time to add a robot.
