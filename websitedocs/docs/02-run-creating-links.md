---
title: Run the Simulation
---


### 1. Run LinkExampleSimulation. 
   You should get a view resembling that of the figure below.
   
![ Six example shapes. Coordinate systems are located at the origin of each shape.](/img/documentation/Black6Figures.PNG)

### 2. Modify the values of the parameters of some of the shapes

   For example, try modifying the value of the sphere:
   ```java
   private static final double SPHERE_R = 0.15;
   ```
   
   Notice how it changes the shape.

   Try modifying the other shapes as well and notice how their size and proportions change with each value.
   
### 3. Try modifying the space set between each shape relative to each shape. 
   The origin of each coordinate system is at the origin of each shape.
   
   Try modifying the `OFFSET` and `COORD_LENGTH` values.
   
   The parameters passed into `translate(OFFSET, 1.0, 0.0)` within the `exampleShapes()` method will change the placement of the shape that calls the `translate()` method.
   
### 4. Add rotations and more translation and notice their effects.  
   In order to add rotations you have to use the built in `linkGraphics.rotate(double rotAng, int rotAxis)` method.
   
   Make sure that you import `us.ihmc.robotics.Axis` in order to use the rotate method.
   
   Try testing this method out on some of your already created shapes.  Notice how they are effected.
   
   For more information you can look at the [LinkGraphicsDescription API](https://ihmcrobotics.github.io/simulation-construction-set/docs/01-links-constructors-and-methods.html) Page.
   
### 5. Try creating some shapes with different appearances
   To change the appearance of some of the shapes take a look at the [YoAppearance Utility API](https://ihmcrobotics.github.io/simulation-construction-set/docs/04-yo-appearance-api.html).
  
   
### 6. Try experimenting to make an object, such as a snowman, out of the shapes.
   Try to create a snowman that looks like the image below.
   
   ![ Three Spheres stacked on top of each other with different values set for `SPHERE_R`](/img/documentation/Snowman.PNG)
   
<details>
<summary> Snowman example code </summary>

```java
package us.ihmc.exampleSimulations.linkExamples;

import us.ihmc.graphicsDescription.appearance.YoAppearance;
import us.ihmc.robotics.robotDescription.LinkGraphicsDescription;
import us.ihmc.simulationconstructionset.Link;
import us.ihmc.simulationconstructionset.Robot;
import us.ihmc.simulationconstructionset.SimulationConstructionSet;



public class LinkExamplesSimulation
{
   private SimulationConstructionSet sim;

   private static final double SPHERE_R = 0.15;

   private static final double OFFSET = 1.5, COORD_LENGTH = 0.5;


   public LinkExamplesSimulation()
   {
      Robot nullRob = null;
      sim = new SimulationConstructionSet(nullRob);
      // position the camera to view links
      sim.setCameraPosition(10.0, 6.0, 3.0);
      sim.setCameraFix(0.5, 0.5, 0.0);
      Link exampleShapes = exampleShapes();
      sim.addStaticLink(exampleShapes);
      sim.setGroundVisible(false);

      Thread myThread = new Thread(sim);
      myThread.start();
   }


   public static void main(String[] args)
   {
      new LinkExamplesSimulation();
   }


   private Link exampleShapes()
   {
      Link ret = new Link("example shapes");
      LinkGraphicsDescription linkGraphics = new LinkGraphicsDescription();

      // Sphere
      linkGraphics.translate(0.0 * OFFSET, 0.0, 0.0);
      linkGraphics.addCoordinateSystem(COORD_LENGTH);
      linkGraphics.addSphere(SPHERE_R + 0.1, YoAppearance.White());

      linkGraphics.translate(0.0 * OFFSET, 0.0, -0.6);
      linkGraphics.addCoordinateSystem(COORD_LENGTH);
      linkGraphics.addSphere(SPHERE_R + 0.3, YoAppearance.White());

      linkGraphics.translate(0.0 * OFFSET, 0.0, -0.9);
      linkGraphics.addCoordinateSystem(COORD_LENGTH);
      linkGraphics.addSphere(SPHERE_R + 0.55, YoAppearance.White());

      ret.setLinkGraphics(linkGraphics);

      return ret;
   }

}
```
</details>