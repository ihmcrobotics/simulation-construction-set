---
title: Creating Links
---

In this tutorial we will create various shapes and add them to a link.
The next figure shows eleven different shapes, each with a coordinate system at their origin point. 

![ Eleven example shapes. Coordinate systems are located at the origin of each shape.](/img/documentation/Figure6Shapes-500-300.png)

*Eleven example shapes. Coordinate systems are located at the origin of each shape.*

### 1. Create a new package with the name linkExamples
  This should have the same folder structure that you used in the Pendulum Example.
  Your new `linkExamples` package should be within your `us.ihmc.exampleSimulations` package. 
  
### 2. Add the class LinkExamplesSimulation 

Create a new class in the `us.ihmc.exampleSimulations.linkExamples` package called `LinkExamplesSimulation`
     
### 3. Fill in LinkExamplesSimulation as follows:  

The following code is where you will be declaring final variables for shapes like a sphere, ellipsoid, cylinder, arc torus, and wedge.

We typically declare these variables separately than the creation of the shapes so that they can can be more easily changed and found.

<pre><code data-url-index="0" data-snippet="portion" data-start="package" data-end="public LinkExamplesSimulation()" id="LinkExamplesSimVariables"></code></pre>

The parameters that adjust placement and spacing of the shapes are the `OFFSET` and `COORD_LENGTH`.

### 4. Add a constructor to your LinkExamplesSimulation class
   The structure of this code should now be familiar to you. Notice, however, that this is purely a static graphic we are creating. There is no robot, and thus no dynamics.  
   This will allow us to focus on creating the graphics of a link.
   
 <pre><code data-url-index="0" data-snippet="portion" data-start="public LinkExamplesSimulation()" data-end="public static void main" id="linkExamplesSimConstructor"></code></pre>
   
### 5. Next add the method exampleShapes to your LinkExampleSimulation class
This method will show you examples of how to create simple shapes, assign a color, and add them to your simulation.

In this method there are examples of a Sphere, Ellipsoid, Cylinder, ArcTorus, Extruded Polygon, Mesh Data, and a Gridded Polytope.

 <pre><code data-url-index="0" data-snippet="portion" data-start="private Link exampleShapes()" data-end="&#10&#10}" id="linkExamplesSimExampleShapes"></code></pre>
   
### 6. Add the main method to your class
This method should just simply create a new `LinkExamplesSimulation`.

Just as follows:
 <pre><code data-url-index="0" data-snippet="portion" data-start="public static void" data-end="&#10&#10" id="linkExamplesSimMain"></code></pre>
   
   
### Full code for the class
<details>
<summary> Link Examples Simulation </summary>
<pre><code data-url-index="0" data-snippet="complete" id="LinkExamplesSimClass"></code></pre>
</details>

<script id="snippetscript" src="../snippetautomation/codesnippets.js" sources=Array.of("https://rawgit.com/ihmcrobotics/ihmc-open-robotics-software/master/example-simulations/src/main/java/us/ihmc/exampleSimulations/linkExamples/LinkExamplesSimulation.java")></script>
