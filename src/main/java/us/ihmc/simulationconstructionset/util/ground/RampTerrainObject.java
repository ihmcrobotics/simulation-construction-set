package us.ihmc.simulationconstructionset.util.ground;

import java.util.ArrayList;

import us.ihmc.euclid.Axis3D;
import us.ihmc.euclid.geometry.BoundingBox3D;
import us.ihmc.euclid.shape.primitives.Ramp3D;
import us.ihmc.euclid.shape.primitives.interfaces.Shape3DReadOnly;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.interfaces.Point3DBasics;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DBasics;
import us.ihmc.graphicsDescription.Graphics3DObject;
import us.ihmc.graphicsDescription.appearance.AppearanceDefinition;
import us.ihmc.graphicsDescription.appearance.YoAppearance;
import us.ihmc.jMonkeyEngineToolkit.HeightMapWithNormals;

public class RampTerrainObject implements TerrainObject3D, HeightMapWithNormals
{
   private final double xMin, xMax, yMin, yMax;
   private final double xStart, xEnd;
   private final double zStart, zEnd;

   private final BoundingBox3D boundingBox;
   
   private Graphics3DObject linkGraphics;
   
   private final ArrayList<Shape3DReadOnly> terrainCollisionShapes = new ArrayList<>();

   public RampTerrainObject(double xStart, double yStart, double xEnd, double yEnd, double height, AppearanceDefinition appearance)
   {
      this(xStart, yStart, xEnd, yEnd, 0, height, appearance);
   }

   public RampTerrainObject(double xStart, double yStart, double xEnd, double yEnd, double zStart, double zEnd, AppearanceDefinition appearance)
   {
      this.xStart = xStart;
      this.xEnd = xEnd;
      this.zStart = zStart;
      this.zEnd = zEnd;

      xMin = Math.min(xStart, xEnd);
      xMax = Math.max(xStart, xEnd);

      yMin = Math.min(yStart, yEnd);
      yMax = Math.max(yStart, yEnd);

      linkGraphics = new Graphics3DObject();
      linkGraphics.translate((xStart + xEnd) / 2.0, (yStart + yEnd) / 2.0, zStart);

      if (xStart > xEnd)
         linkGraphics.rotate(Math.PI, Axis3D.Z);
      linkGraphics.addWedge(Math.abs(xEnd - xStart), Math.abs(yEnd - yStart), zEnd-zStart, appearance);
      
      Point3D minPoint = new Point3D(xMin, yMin, Double.NEGATIVE_INFINITY);
      Point3D maxPoint = new Point3D(xMax, yMax, zEnd);
      
      boundingBox = new BoundingBox3D(minPoint, maxPoint);

      RigidBodyTransform transform = new RigidBodyTransform();
      transform.appendTranslation((xStart + xEnd) / 2.0, (yStart + yEnd) / 2.0, zStart);

      if (xStart > xEnd)
         transform.appendYawRotation(Math.PI);

      Ramp3D ramp3DShape = new Ramp3D(transform, Math.abs(xEnd - xStart), Math.abs(yEnd - yStart), Math.abs(zEnd - zStart));
      terrainCollisionShapes.add(ramp3DShape);
   }

   public RampTerrainObject(double xStart, double yStart, double xEnd, double yEnd, double height)
   {
      this(xStart, yStart, xEnd, yEnd, 0.0, height, YoAppearance.Black());
   }

   public RampTerrainObject(double xStart, double yStart, double xEnd, double yEnd, double zStart, double zEnd)
   {
      this(xStart, yStart, xEnd, yEnd, zStart, zEnd, YoAppearance.Black());
   }

   @Override
   public Graphics3DObject getLinkGraphics()
   {
      return linkGraphics;
   }

   @Override
   public double heightAndNormalAt(double x, double y, double z, Vector3DBasics normalToPack)
   {
      double heightAt = heightAt(x, y, z);
      surfaceNormalAt(x, y, z, normalToPack);
      return heightAt;
   }

   @Override
   public double heightAt(double x, double y, double z)
   {
      if ((x > xMin) && (x < xMax) && (y > yMin) && (y < yMax))
      {
         return zStart + (x - xStart) / (xEnd - xStart) * (zEnd - zStart);
      }

      return 0.0;
   }

   public void surfaceNormalAt(double x, double y, double z, Vector3DBasics normal)
   {
      double threshhold = 0.015;
      normal.setX(0.0);
      normal.setY(0.0);
      normal.setZ(1.0);

      if ((x < xMin) || (x > xMax) || (y < yMin) || (y > yMax) || (z > zEnd))
         return;

         /*
       * if (Math.abs(x-xMin) < threshhold) { normal.x = -1.0;normal.y = 0.0;normal.z = 0.0; }
          */

      else if (z > heightAt(x, y, z) - threshhold)
      {
         normal.setX(zEnd - zStart);
         normal.setY(0.0);
         normal.setZ(xStart - xEnd);

         normal.normalize();
         if (normal.getZ() < 0.0)
            normal.scale(-1.0);
      }

      else if (Math.abs(x - xEnd) < threshhold)
      {
         if (xEnd > xStart)
            normal.setX(1.0);
         else
            normal.setX(-1.0);
         normal.setY(0.0);
         normal.setZ(0.0);
      }

      else if (Math.abs(y - yMin) < threshhold)
      {
         normal.setX(0.0);
         normal.setY(-1.0);
         normal.setZ(0.0);
      }

      else if (Math.abs(y - yMax) < threshhold)
      {
         normal.setX(0.0);
         normal.setY(1.0);
         normal.setZ(0.0);
      }
   }

   public void closestIntersectionTo(double x, double y, double z, Point3DBasics intersection)
   {
      intersection.setX(x);    // Go Straight Up for now...
      intersection.setY(y);
      intersection.setZ(heightAt(x, y, z));
   }

   public void closestIntersectionAndNormalAt(double x, double y, double z, Point3DBasics intersection, Vector3DBasics normal)
   {
      intersection.setX(x);    // Go Straight Up for now...
      intersection.setY(y);
      intersection.setZ(heightAt(x, y, z));

      surfaceNormalAt(x, y, z, normal);
   }

   @Override
   public boolean checkIfInside(double x, double y, double z, Point3DBasics intersectionToPack, Vector3DBasics normalToPack)
   {
      double heightAt = heightAt(x, y, z);
      if (z > heightAt)
         return false;

      intersectionToPack.set(x, y, heightAt); 
      surfaceNormalAt(x, y, z, normalToPack);
      
      return true;
   }
   
   @Override
   public boolean isClose(double x, double y, double z)
   {
      return boundingBox.isXYInsideInclusive(x, y);
   }

   public double getXMin()
   {
      return xMin;
   }

   public double getYMin()
   {
      return yMin;
   }

   public double getXMax()
   {
      return xMax;
   }

   public double getYMax()
   {
      return yMax;
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