package us.ihmc.simulationconstructionset.util.ground;

import java.util.ArrayList;
import java.util.List;

import us.ihmc.euclid.Axis3D;
import us.ihmc.euclid.geometry.BoundingBox3D;
import us.ihmc.euclid.geometry.Line3D;
import us.ihmc.euclid.shape.primitives.Box3D;
import us.ihmc.euclid.shape.primitives.Cylinder3D;
import us.ihmc.euclid.shape.primitives.interfaces.Shape3DReadOnly;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple3D.interfaces.Point3DBasics;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DBasics;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;
import us.ihmc.graphicsDescription.Graphics3DObject;
import us.ihmc.graphicsDescription.appearance.AppearanceDefinition;
import us.ihmc.jMonkeyEngineToolkit.HeightMapWithNormals;

public class CylinderTerrainObject implements TerrainObject3D, HeightMapWithNormals
{
   protected final BoundingBox3D boundingBox;
   protected final Cylinder3D cylinder;
   private final RigidBodyTransform location;
   private final double height;
   private final double radius;
   protected Graphics3DObject linkGraphics;

   private final Point3D tempPoint = new Point3D();
   private final Vector3D zVector = new Vector3D(0.0, 0.0, -1.0);

   private final ArrayList<Shape3DReadOnly> terrainCollisionShapes = new ArrayList<>();

   // TODO: change box based surface equations to cylinder surface equations

   public CylinderTerrainObject(RigidBodyTransform location, double height, double radius, AppearanceDefinition appearance)
   {
      this.height = height;
      this.radius = radius;
      this.location = location;
      cylinder = new Cylinder3D(height, radius);
      cylinder.applyTransform(location);

      Box3D box = new Box3D(location, radius * 2, radius * 2, height);
      Point3DBasics[] vertices = box.getVertices();
      Point3D minPoint = new Point3D(Double.POSITIVE_INFINITY, Double.POSITIVE_INFINITY, Double.POSITIVE_INFINITY);
      Point3D maxPoint = new Point3D(Double.NEGATIVE_INFINITY, Double.NEGATIVE_INFINITY, Double.NEGATIVE_INFINITY);

      for (Point3DBasics cornerPoint : vertices)
      {
         for (int i = 0; i < 3; i++)
         {
            double coordinate = cornerPoint.getElement(i);
            if (coordinate > maxPoint.getElement(i))
               maxPoint.setElement(i, coordinate);
            if (coordinate < minPoint.getElement(i))
               minPoint.setElement(i, coordinate);
         }
      }

      boundingBox = new BoundingBox3D(minPoint, maxPoint);

      addGraphics(appearance);

      Cylinder3D cylinderShape = new Cylinder3D(height, radius);
      cylinderShape.applyTransform(location);

      terrainCollisionShapes.add(cylinderShape);
   }

   public CylinderTerrainObject(Vector3DReadOnly center, double pitchDownDegrees, double yawDegrees, double height, double radius, AppearanceDefinition app)
   {
      this(yawPitchDegreesTransform(center, yawDegrees, pitchDownDegrees), height, radius, app);
   }

   private static RigidBodyTransform yawPitchDegreesTransform(Vector3DReadOnly center, double yawCCWDegrees, double pitchDownDegrees)
   {
      RigidBodyTransform location = new RigidBodyTransform();
      location.getRotation().setYawPitchRoll(Math.toRadians(yawCCWDegrees), Math.toRadians(pitchDownDegrees), 0.0);
      location.getTranslation().set(center);
      return location;
   }

   protected void addGraphics(AppearanceDefinition appearance)
   {
      RigidBodyTransform transform = transformToBottomOfCylinder();

      linkGraphics = new Graphics3DObject();
      linkGraphics.transform(transform);

      getLinkGraphics().addCylinder(height, radius, appearance);
   }

   private RigidBodyTransform transformToBottomOfCylinder()
   {
      RigidBodyTransform ret = new RigidBodyTransform(location);
      ret.appendTranslation(0.0, 0.0, -height / 2.0);
      return ret;
   }

   @Override
   public Graphics3DObject getLinkGraphics()
   {
      return linkGraphics;
   }

   @Override
   public BoundingBox3D getBoundingBox()
   {
      return boundingBox;
   }

   @Override
   public double heightAndNormalAt(double x, double y, double z, Vector3DBasics normalToPack)
   {
      double heightAt = heightAt(x, y, 1e9);
      surfaceNormalAt(x, y, heightAt, normalToPack);

      return heightAt;
   }

   @Override
   public double heightAt(double x, double y, double z)
   {
      Point3D testPoint = new Point3D(x, y, z);
      Line3D zLine = new Line3D(testPoint, zVector);

      Point3D intersection1 = new Point3D();
      Point3D intersection2 = new Point3D();
      int numberOfIntersections = cylinder.intersectionWith(zLine, intersection1, intersection2);

      if (numberOfIntersections == 0)
         return 0;
      else if (numberOfIntersections == 1)
         return intersection1.getZ();
      else
      {
         /*
          * TODO Review the following. I think it should return only the highest of the two intersections
          * which always is intersection1.
          */
         if (testPoint.distanceSquared(intersection1) < testPoint.distanceSquared(intersection2))
            return intersection1.getZ();
         else
            return intersection2.getZ();
      }
   }

   public Line3D getAxis()
   {
      Point3D axisOrigin = new Point3D();
      axisOrigin.set(location.getTranslation());

      Vector3D axisDirection = getAxisDirectionCopy();

      return new Line3D(axisOrigin, axisDirection);
   }

   public Vector3D getAxisDirectionCopy()
   {
      Vector3D axisDirection = new Vector3D();
      location.getRotation().getColumn(Axis3D.Z.ordinal(), axisDirection);
      return axisDirection;
   }

   public double getXMin()
   {
      return boundingBox.getMinX();
   }

   public double getYMin()
   {
      return boundingBox.getMinY();
   }

   public double getXMax()
   {
      return boundingBox.getMaxX();
   }

   public double getYMax()
   {
      return boundingBox.getMaxY();
   }

   @Override
   public boolean isClose(double x, double y, double z)
   {
      return boundingBox.isXYInsideInclusive(x, y);
   }

   private final Point3D ignoreIntesectionPoint = new Point3D();
   private final Vector3D ignoreNormal = new Vector3D();

   public void closestIntersectionTo(double x, double y, double z, Point3DBasics intersectionToPack)
   {
      tempPoint.set(x, y, z);
      cylinder.evaluatePoint3DCollision(tempPoint, intersectionToPack, ignoreNormal);
   }

   public void surfaceNormalAt(double x, double y, double z, Vector3DBasics normalToPack)
   {
      tempPoint.set(x, y, z);
      cylinder.evaluatePoint3DCollision(tempPoint, ignoreIntesectionPoint, normalToPack);
   }

   public void closestIntersectionAndNormalAt(double x, double y, double z, Point3DBasics intersectionToPack, Vector3DBasics normalToPack)
   {
      tempPoint.set(x, y, z);
      cylinder.evaluatePoint3DCollision(tempPoint, intersectionToPack, normalToPack);
   }

   @Override
   public boolean checkIfInside(double x, double y, double z, Point3DBasics intersectionToPack, Vector3DBasics normalToPack)
   {
      tempPoint.set(x, y, z);
      return cylinder.evaluatePoint3DCollision(tempPoint, intersectionToPack, normalToPack);
   }

   @Override
   public HeightMapWithNormals getHeightMapIfAvailable()
   {
      return this;
   }

   @Override
   public List<Shape3DReadOnly> getTerrainCollisionShapes()
   {
      return terrainCollisionShapes;
   }
}
