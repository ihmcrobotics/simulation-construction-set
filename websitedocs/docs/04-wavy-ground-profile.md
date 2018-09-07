---
title: Create a New Class WavyGroundProfile
---

### Lastly Create a Class Named `WavyGroundProfile`
   Fill the class with the following:

```java
package us.ihmc.exampleSimulations.fallingBrick;
import javax.vecmath.Point3d;
import javax.vecmath.Vector3d;
import us.ihmc.jMonkeyEngineToolkit.GroundProfile3D;
import us.ihmc.jMonkeyEngineToolkit.HeightMapWithNormals;
import us.ihmc.robotics.geometry.BoundingBox3d;
 
public class WavyGroundProfile implements GroundProfile3D, HeightMapWithNormals
{
   private double xMin = -2.0, xMax = 2.0, yMin = -2.0, yMax = 2.0, zMin = -10.0, zMax = 10.0;
    
   private BoundingBox3d boundingBox = new BoundingBox3d(new Point3d(xMin, yMin, zMin), new Point3d(xMax, yMax, zMax));
   public WavyGroundProfile()
   {
   }
   public double heightAndNormalAt(double x, double y, double z, Vector3d normalToPack)
   {
      double heightAt = heightAt(x, y, z);
      surfaceNormalAt(x, y, heightAt, normalToPack);
      return heightAt;
   }
    
   public HeightMapWithNormals getHeightMapIfAvailable()
   {
      return this;
   }
}

```