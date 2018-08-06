---
title: SimplePendulumRobot.java
---

Now that you have the simulation class up and running it's time to add a robot.  Robots in SCS extend the `us.ihmc.simulationconstructionset.Robot` class and are made up of joints and links. The class Robot is included in Simulation Construction Set and has built in graphics, dynamics, etc. Extending the class is an easy way to make a new type of robot.

To start with, we will create one of the simplest robots possible: a Pendulum.  

### Description of the robot
  
*"A pendulum is a weight suspended from a pivot so that it can swing freely.
When a pendulum is displaced sideways from its resting, equilibrium position, it is subject to a restoring force due to gravity that will accelerate it back toward the equilibrium position.
When released, the restoring force combined with the pendulum's mass causes it to oscillate about the equilibrium position, swinging back and forth. 
The time for one complete cycle, a left swing and a right swing, is called the period. The period depends on the length of the pendulum and also to a slight degree on the amplitude, the width of the pendulum's swing."*  
Source: [Wikipedia](https://en.wikipedia.org/wiki/Pendulum)


![pendulum](/img/scs-tutorials/simple-pendulum/pendulum.png) 

In order to build this pendulum for our simulation, we will use two SCS objects: one **Link** and one **Joint**. 

#### Links and Joints
The **Link** object describes the physical properties (*Mass*, *Center of Mass* offset vector, *Moment of Intertia*, ...) of a rigid body.

The **Joint** object is used to model the connections between links. Joints describes the motion constraint between Links and the physics simulation. 
SCS provides a variety of Joints used to build robots such as: FloatingJoint, FreeJoint, FloatingPlanarJoint, PinJoint, SliderJoint...   

To represent the pendulum's pivot, we will use a **PinJoint** which is a rotational joint with a single degree of freedom. Pin joints allow rotation around a single axis specified upon creation (the Y-axis in this case).   

### 1. Create the SimplePendulumRobot Class

Create a new class in the `us.ihmc.exampleSimulations.simplePendulum` package called `SimplePendulumRobot` that extends `us.ihmc.simulationconstructionset.Robot`.
   
Your class should look like this:

```java
package us.ihmc.exampleSimulations.simplePendulum;

import us.ihmc.simulationconstructionset.Robot;

public class SimplePendulumRobot extends Robot // SimplePendulumRobot inherits some properties and methods from Robot class
{

}
```

### 2. Define the Pendulum Constants and Initial Values:
 
<details open>
<summary> Constants and Initial Values </summary>
<pre><code data-url-index="0" data-snippet="portion" id="SimplePendulumRobotVariables" data-start="public class SimplePendulumRobot" data-end="/*&#10      Define its constructor"></code></pre>
</details>

***Lengths are expressed in meters (m), masses in kilograms (kg)**

### 3. Include Required Imports:

Before continuing, make sure you have added the following classes to your imports:  
<details open>
<summary> Required Imports </summary>
<pre><code data-url-index="0" data-snippet="portion" id="SimplePendulumRobotImports" data-start="import" data-end="&#10&#10"></code></pre>
</details>   

### 4. Define the Constructor of the Robot:

<details open>
<summary> Constructor: SimplePendulumRobot() </summary>

```java
   /*
       Constructor creates an instance of the class SimplePendulumRobot
   */
   public SimplePendulumRobot()
   {
      // a. Call parent class "Robot" constructor. The string "SimplePendulum" will be the name of the robot.  
      super("pendulum");
   }
```
</details>   

* **Call Parent Constructor:**  
`super("pendulum");` creates an instance of the class Robot named "pendulum" in the SCS system.

### 5. Create the Pendulum's Rod Using a Link and the 3D Graphics which Represent the Link and Joint in SCS:

Add the following method to your class:
<details open>
<summary> pendulumLink Method </summary>
<pre><code data-url-index="0" data-snippet="portion" id="pendulumLink" data-start="private Link pendulumLink()" data-end="&#10&#10}"></code></pre>
</details>  

Add the following to the end of the constructor:
<details open>
<summary> Create pendulumLink </summary> 
<pre><code data-url-index="0" data-snippet="portion" id="pendulumLinkCreate" data-start="fulcrumPinJoint.setLink" data-end="&#10"></code></pre>
</details>  

* **Create a new Link:**  
`Link pendulumLink = new Link("PendulumLink");`  
This line creates a new Link named "PendulumLink".

* **Set its physical properties:**  
`pendulumLink.setMass(BALL_MASS);`  
This line sets the mass of the link to 1.0 kg.  
`pendulumLink.setComOffset(0.0, 0.0, -ROD_LENGTH);`  
This line sets the center of mass offset of the link to be located at the tip of the rod which is 1.0m opposite the pivot joint.  
`pendulumLink.setMomentOfInertia(0.0, FULCRUM_MOMENT_OF_INERTIA_ABOUT_Y, 0.0);`  
This line sets the moment of inertia about the Y axis. Note that the moment of inertia is defined about the center of mass.
Therefore, if the moment of inertia is set to zero, the link will be a point mass.  

* **Create a new LinkGraphicsDescription:**  
`LinkGraphicsDescription pendulumGraphics = new LinkGraphicsDescription();`  
This line creates a new instance of the LinkGraphicsDescription class which, once attached to its `LinkGraphics`, is used to visually represent links in the SCS 3D view. 
 
* **Add the pivot:**  
`pendulumGraphics.addSphere(FULCRUM_RADIUS, YoAppearance.BlueViolet());`  
Here we add a purple sphere 3D object to `pendulumGraphics`. Since no transformation has been applied to this graphic component the sphere's position is (0.0, 0.0, 0.0). This is used represent the pivot/fulcrum of the pendulum.  

* **Add the rod:**  
`pendulumGraphics.translate(0.0, 0.0, -ROD_LENGTH);`  
Translates from the current position by the specified distances. Graphic components added after translation will appear in the new location. The coordinate system for these translations is based on those that preceded it.   
`pendulumGraphics.addCylinder(ROD_LENGTH, ROD_RADIUS, YoAppearance.Black());`  
Adds a 1m long black cylinder representing the rod.  

* **Add the ball:**  
`pendulumGraphics.addSphere(BALL_RADIUS, YoAppearance.Chartreuse());`  
Adds a yellow sphere representing the ball at the end of the rod.   

* **Attach the LinkGraphicsDescription to its Link:**  
`pendulumLink.setLinkGraphics(pendulumGraphics);`  
This line associates our `pendulumGraphics` object with the `pendulumLink` object and in doing so, translates and rotate the graphic components to be in the same frame of reference as the `pendulumLink`.   
 

### 6. Create the Pendulum's Pivot or fulcrum Using a PinJoint:

Add the following to the end of the constructor:
<details open>
<summary> Create: fulcrumPinJoint </summary>
<pre><code data-url-index="0" data-snippet="portion" id="fulcrumPinJoint" data-start="// b. Add a joint" data-end="}"></code></pre>
</details>   

* **Create a pin joint:**   
`PinJoint fulcrumPinJoint = new PinJoint("FulcrumPinJoint", new Vector3d(0.0, 0.0, 1.5), this, Axis.Y);`   
The first parameter "FulcrumPinJoint" is the name of the joint and will be used in all the variables associated with that joint.  
The second parameter "new Vector3d(0.0, 0.0, 1.5)" defines the offset of this joint from the previous joint.  
Since we want to position the fulcrum of the pendulum at a height of 1.5 meters above the ground, the default vector (0.0, 0.0, 1.5) will be used.   
The third parameter "this" refers to the robot itself. The final parameter "Axis.Y" defines the axis of rotation for this pin joint. 

* **Set some properties for this joint:**  
`fulcrumPinJoint.setInitialState(fulcrumInitialPositionRadians, fulcrumInitialVelocity);`  
Initial state of the pin joint. The pendulum will start its course from an horizontal position with no initial speed.  
`fulcrumPinJoint.setDamping(0.3);`  
Add some damping to force the pendulum ball to converge faster to equilibrium position. 
 
* **Finally, attach a Link to this joint:**   
`fulcrumPinJoint.setLink(pendulumLink);`  
Set the link created previously as the link for this joint.   

* **Add a RootJoint to the robot:**  
`this.addRootJoint(fulcrumPinJoint);`  
This line adds `fulcrumPinJoint` as a rootJoint of the robot. 
In order to be part of a robot, a joint must either be added as a root joint, or be attached to a parent joint. 
This ensures the tree structure (forest structure if there are multiple root joints) of the robot.


![note](/img/attention-40.png) **Note on reference frames:**
*In SCS, like the URDF robot description file format, when a joint is created its reference frame is calculated relative to the previous joint's reference frame.
One exception is that joints added as **root joints** of a robot are positioned relative to the world reference frame. The reference frame of each `Link` attached to a `Joint` is identical to the reference frame of the `Joint`.* 

### 8. Add the Robot to the Simulation

Now that you have a robot class, it needs to be instantiated in the simulation class and added to SCS.

In the `SimplePendulumSimulation` class constructor, replace this line

```java
      sim = new SimulationConstructionSet(parameters);
```

with

```java
      SimplePendulumRobot robot = new SimplePendulumRobot();
    
      sim = new SimulationConstructionSet(robot, parameters);
```


### Full code for the class:  
<details>
<summary> Simple Pendulum Robot </summary>
<pre><code data-url-index="0" data-snippet="complete" id="SimplePendulumRobotClass"></code></pre>
</details>

<script id="snippetscript" src="../snippetautomation/codesnippets.js" sources=Array.of("https://rawgit.com/ihmcrobotics/ihmc-open-robotics-software/master/example-simulations/src/main/java/us/ihmc/exampleSimulations/simplePendulum/SimplePendulumRobot.java")></script>


 
