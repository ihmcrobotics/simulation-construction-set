package us.ihmc.simulationconstructionset.util.ground;

import us.ihmc.euclid.geometry.BoundingBox3D;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.graphicsDescription.Graphics3DObject;
import us.ihmc.graphicsDescription.appearance.AppearanceDefinition;
import us.ihmc.graphicsDescription.appearance.YoAppearance;
import us.ihmc.jMonkeyEngineToolkit.HeightMapWithNormals;

public class ConeTerrainObject implements TerrainObject3D, HeightMapWithNormals
{
   private final double xMin, xMax, yMin, yMax;
   private final double xMiddle, yMiddle, bottomRadius, topRadius;
   private final double height;

   private final BoundingBox3D boundingBox;

   private Graphics3DObject linkGraphics;

   public ConeTerrainObject(double xMiddle, double yMiddle, double bottomRadius, double topRadius, double height, AppearanceDefinition appearance)
   {
      this.xMiddle = xMiddle;
      this.yMiddle = yMiddle;
      this.bottomRadius = bottomRadius;
      this.topRadius = topRadius;
      this.height = height;

      xMin = xMiddle - bottomRadius;
      xMax = xMiddle + bottomRadius;

      yMin = yMiddle - bottomRadius;
      yMax = yMiddle + bottomRadius;

      linkGraphics = new Graphics3DObject();
      linkGraphics.translate(xMiddle, yMiddle, 0.0);
      linkGraphics.addGenTruncatedCone(height, bottomRadius, bottomRadius, topRadius, topRadius, appearance);

      Point3D minPoint = new Point3D(xMin, yMin, Double.NEGATIVE_INFINITY);
      Point3D maxPoint = new Point3D(xMax, yMax, height);

      boundingBox = new BoundingBox3D(minPoint, maxPoint);
   }

   public ConeTerrainObject(double xMiddle, double yMiddle, double bottomRadius, double topRadius, double height)
   {
      this(xMiddle, yMiddle, bottomRadius, topRadius, height, YoAppearance.Red());
   }

   @Override
   public Graphics3DObject getLinkGraphics()
   {
      return linkGraphics;
   }

   @Override
   public double heightAndNormalAt(double x, double y, double z, Vector3D normalToPack)
   {
      double heightAt = heightAt(x, y, z);
      surfaceNormalAt(x, y, z, normalToPack);
      return heightAt;
   }

   @Override
   public double heightAt(double x, double y, double z)
   {
      double r_from_center = Math.sqrt((x - xMiddle) * (x - xMiddle) + (y - yMiddle) * (y - yMiddle));
      if (r_from_center > bottomRadius)
         return 0.0;

      if (r_from_center < topRadius)
         return height;
      else
      {
         return (1.0 - ((r_from_center - topRadius) / (bottomRadius - topRadius)) * height);
      }

   }

   public void surfaceNormalAt(double x, double y, double z, Vector3D normal)
   {
      normal.setX(0.0);
      normal.setY(0.0);
      normal.setZ(1.0);
   }

   public void closestIntersectionTo(double x, double y, double z, Point3D intersection)
   {
      intersection.setX(x); // Go Straight Up for now...
      intersection.setY(y);
      intersection.setZ(heightAt(x, y, z));
   }

   public void closestIntersectionAndNormalAt(double x, double y, double z, Point3D intersection, Vector3D normal)
   {
      intersection.setX(x); // Go Straight Up for now...
      intersection.setY(y);
      intersection.setZ(heightAt(x, y, z));

      surfaceNormalAt(x, y, z, normal);
   }

   @Override
   public boolean checkIfInside(double x, double y, double z, Point3D intersectionToPack, Vector3D normalToPack)
   {
      intersectionToPack.setX(x); // Go Straight Up for now...
      intersectionToPack.setY(y);
      intersectionToPack.setZ(heightAt(x, y, z));

      surfaceNormalAt(x, y, z, normalToPack);

      return (z < intersectionToPack.getZ());
   }

   @Override
   public boolean isClose(double x, double y, double z)
   {
      if ((x < xMin) || (x > xMax) || (y < yMin) || (y > yMax))
         return false;
      if (z > height)
         return false;

      return true;
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
