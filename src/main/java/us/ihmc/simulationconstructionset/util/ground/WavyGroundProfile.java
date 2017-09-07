package us.ihmc.simulationconstructionset.util.ground;

import us.ihmc.euclid.geometry.BoundingBox3D;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.jMonkeyEngineToolkit.GroundProfile3D;
import us.ihmc.jMonkeyEngineToolkit.HeightMapWithNormals;


public class WavyGroundProfile implements GroundProfile3D, HeightMapWithNormals
{
   private double xMin = -2.0, xMax = 2.0, yMin = -2.0, yMax = 2.0, zMin = -10.0, zMax = 10.0;
   
   private BoundingBox3D boundingBox = new BoundingBox3D(new Point3D(xMin, yMin, zMin), new Point3D(xMax, yMax, zMax));

   public WavyGroundProfile()
   {
   }

   @Override
   public double heightAndNormalAt(double x, double y, double z, Vector3D normalToPack)
   {
      double heightAt = heightAt(x, y, z);
      surfaceNormalAt(x, y, heightAt, normalToPack);
      return heightAt;
   }
   
   @Override
   public double heightAt(double x, double y, double z)
   {
      if ((x > xMin) && (x < xMax) && (y > yMin) && (y < yMax))
         return 1.0 * Math.exp(-Math.abs(2.0 * x)) * Math.exp(-Math.abs(2.0 * y)) * Math.sin(2.0 * Math.PI * 0.7 * x);
      else
         return 0.0;
   }

   public void surfaceNormalAt(double x, double y, double z, Vector3D normal)
   {
      normal.setX(0.0);
      normal.setY(0.0);
      normal.setZ(1.0);
   }

   public void closestIntersectionTo(double x, double y, double z, Point3D point)
   {
      point.setX(x);
      point.setY(y);
      point.setZ(heightAt(x, y, z));
   }

   public void closestIntersectionAndNormalAt(double x, double y, double z, Point3D point, Vector3D normal)
   {
      closestIntersectionTo(x, y, z, point);
      surfaceNormalAt(x, y, z, normal);
   }

   @Override
   public boolean checkIfInside(double x, double y, double z, Point3D intersectionToPack, Vector3D normalToPack)
   {
      closestIntersectionTo(x, y, z, intersectionToPack);
      surfaceNormalAt(x, y, z, normalToPack);
      
      return (z < intersectionToPack.getZ());
   }
   
   @Override
   public boolean isClose(double x, double y, double z)
   {
      return boundingBox.isInsideInclusive(x, y, z);
   }

   @Override
   public BoundingBox3D getBoundingBox()
   {
      return boundingBox;
   }

   @Override
   public HeightMapWithNormals getHeightMapIfAvailable()
   {
      return this;
   }

}

