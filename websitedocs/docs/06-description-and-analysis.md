---
title: Description and Analysis
---

###  1. Examine the FlyballGovernorRobot source code
   Add the following constructor to your FlyballGovernorRobot class.  
   Note where the ExternalForcePoints are created and attached.  
   
<pre><code data-url-index="1" data-snippet="portion" data-start="public FlyballGovernorRobot" data-end="public ExternalForcePoint" id="RobotConstructor"></code></pre>
   
### 2. Add the link methods to the FlyballGovernorRobot class

   These methods build the links for the simulation.
<pre><code data-url-index="1" data-snippet="portion" data-start="private Link centerRod()" data-end="private YoDouble" id="RobotMethods"></code></pre>

###  3. Add the doConstraint method in FlyballGovernorSimpleClosedLoopConstraint
<pre><code data-url-index="2" data-snippet="portion" data-start="private void doConstraint" data-end="public YoVariableRegistry" id="doConstraint"></code></pre>
   
   Here is where the constraint forces between two points, A and B, are generated. We see that for each constraint, a linear spring-damper is used to "glue" the two ExternalForcePoints together.

###  4. Run the FlyballGovernorSimulation class
   Note that the blue cylinder rises as the device spins faster. To vary the desired speed, change the value of `q_d_cylinder_z`.

###  5. Change the value of constraintGain and constraintDamp to 0.0 while in simulation
   See that the constraint is no longer enforced and the "glue" joint comes apart.
  
###  6. Now look at the doControl() function in FlyballGovernorRobot
   The feedback mechanism used in the `FlyballGovernor` is `tau_rotation.set(k_feedback.getDoubleValue() * (q_d_cylinder_z.getDoubleValue() - q_cylinder_z.getDoubleValue()));` 
   The torque on the rotation joint is proportional to the height of the cylinder. This is an example of how flyballs on locomotive engines are used. Flyball governors throttle steam engines proportionally to the height of the cylinder. In essence, a flyball governor provides velocity feedback control completely mechanically.

###  7. Try implementing a closed-loop mechanism on your own
   Examples include four-bar linkages or a necklace with rigid links.
   
###  Full Code for Classes
<details>
<summary>FlyballGovernorSimulation</summary>
<pre><code data-url-index="0" data-snippet="complete" id="Simulation"></code></pre>
</details>

<details>
<summary>FlyballGovernorRobot</summary>
<pre><code data-url-index="1" data-snippet="complete" id="Robot"></code></pre>
</details>

<details>
<summary>FlyballGovernorSimpleClosedLoopConstraintController</summary>
<pre><code data-url-index="2" data-snippet="complete" id="SimpleClosedLoopConstraintController"></code></pre>
</details>

<details>
<summary>FlyballGovernorCommonControlParameters</summary>
<pre><code data-url-index="3" data-snippet="complete" id="CommonControlParameters"></code></pre>
</details>

<script id="snippetscript" src="../snippetautomation/codesnippets.js" sources=Array.of("https://rawgit.com/ihmcrobotics/ihmc-open-robotics-software/develop/example-simulations/src/main/java/us/ihmc/exampleSimulations/flyballGovernor/FlyballGovernorSimulation.java","https://rawgit.com/ihmcrobotics/ihmc-open-robotics-software/develop/example-simulations/src/main/java/us/ihmc/exampleSimulations/flyballGovernor/FlyballGovernorRobot.java","https://rawgit.com/ihmcrobotics/ihmc-open-robotics-software/develop/example-simulations/src/main/java/us/ihmc/exampleSimulations/flyballGovernor/FlyballGovernorSimpleClosedLoopConstraintController.java","https://rawgit.com/ihmcrobotics/ihmc-open-robotics-software/develop/example-simulations/src/main/java/us/ihmc/exampleSimulations/flyballGovernor/FlyballGovernorCommonControllerParameters.java")></script>
