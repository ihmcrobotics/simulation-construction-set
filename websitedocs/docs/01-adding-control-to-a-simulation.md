---
title: Adding control to a simulation
---


One can add control capabilities to any Robot by setting a RobotController via the `setController(RobotController aRobotController)` method.

### 1. Create the SimplePendulumController Class  

Create a new class in the `us.ihmc.exampleSimulations.simplePendulum` package called `SimplePendulumController` that implements `us.ihmc.simulationconstructionset.robotController.RobotController`.

Your class should look like this:

```java
package us.ihmc.exampleSimulations.simplePendulum;

import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.robotics.robotController.RobotController;

public class SimplePendulumController implements RobotController
{ 
   @Override public void initialize(){}
       
   @Override public void doControl(){}
      
   @Override public YoVariableRegistry getYoVariableRegistry(){}
   
   @Override public String getName(){}
   
   @Override public String getDescription(){}
   
}
```

* **Interface Implementations**  
 
   The `RobotController` interface requires you to implement:  
   - `initialize()` Not used at this point.   
   - `doControl()` This method gets called each update, that's where your control code is written.
   - `getYoVariableRegistry()` returns a YoVariableRegistry object that you need to instantiate in the controller. This object will give you access to the control variables.   
   - `getName()` and `getDescription()` not used at this point.
       


### 2. Add properties and create a constructor for SimplePendulumController

We will start by populating the class with the following elements: 

<pre><code data-url-index="0" data-snippet="portion" data-start="//" data-end="@Override" id="controllerProperties"></code></pre>


### 3. Add some accessors to the SimplePendulumRobot

In the `SimplePendulumRobot` class; add the following lines of code so that one can have access to and set fulcrum joint properties:  

Above the constructor add some variables to store the current state of the fulcrum joint.

```java

/* Some joint state variables */
   private YoDouble tau_fulcrum, q_fulcrum, qd_fulcrum; // Respectively Torque, Position, Velocity

```

![note](/img/attention-40.png) **Note on naming:** *By convention, adding a prefix of 'q', 'qd', and 'tau', refers to the joint angle, joint velocity, and joint torque, respectively.
E.g.  'q_fulcrum',  'qd_fulcrum', 'tau_fulcrum'.*

Add some more lines of code to the constructor to access to the joint's properties:

```java
  
   public SimplePendulumRobot()
   {
      ...
      // Hold references to some properties of this joint using DoubleYoVariables
      q_fulcrum = fulcrumPinJoint.getQYoVariable();
      qd_fulcrum = fulcrumPinJoint.getQDYoVariable();
      tau_fulcrum = fulcrumPinJoint.getTauYoVariable();
   }
```

Add some accessor methods to the pendulum robot.
<pre><code data-url-index="1" data-snippet="portion" data-start="/**&#10    * Fulcrum's angular velocity" data-end="/**&#10    * Create the first link for the DoublePendulumRobot." id="robotMethods"></code></pre>

Now that we have access to these variables, we can use them in our doControl() method.
<pre><code data-url-index="0" data-snippet="portion" data-start="private double positionError" data-end="@Override public YoVariableRegistry" id="robotMethods"></code></pre>
-
### 4. Link the controller to the robot 

In order for the control code to be accessed, it must be set in Simulation Construction Set. 
Therefore, in the `SimplePendulumSimulation` class add the following line of code under your robot's instantiation line:

```java
   public SimplePendulumSimulation()
   {
      SimplePendulumRobot robot = new SimplePendulumRobot();
      robot.setController(new SimplePendulumController(robot)); // Add this line
    
    ...
    
```


### Full code for the class:

<details>
<summary> Simple Pendulum Controller </summary>
<pre><code data-url-index="0" data-snippet="complete" id="ControllerClass"></code></pre>
</details>

<details>
<summary> Simple Pendulum Robot </summary>
<pre><code data-url-index="1" data-snippet="complete" id="RobotClass"></code></pre>
</details>

<details>
<summary> Simple Pendulum Simulation </summary>
<pre><code data-url-index="2" data-snippet="complete" id="SimulationClass"></code></pre>
</details>

<script id="snippetscript" src="../snippetautomation/codesnippets.js" sources=Array.of("https://rawgit.com/ihmcrobotics/ihmc-open-robotics-software/master/example-simulations/src/main/java/us/ihmc/exampleSimulations/simplePendulum/SimplePendulumController.java","https://rawgit.com/ihmcrobotics/ihmc-open-robotics-software/master/example-simulations/src/main/java/us/ihmc/exampleSimulations/simplePendulum/SimplePendulumRobot.java","https://rawgit.com/ihmcrobotics/ihmc-open-robotics-software/master/example-simulations/src/main/java/us/ihmc/exampleSimulations/simplePendulum/SimplePendulumSimulation.java")></script>
