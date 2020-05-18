package us.ihmc.simulationconstructionset.physics.collision.simple;

import us.ihmc.euclid.geometry.BoundingBox3D;
import us.ihmc.euclid.geometry.tools.EuclidGeometryTools;
import us.ihmc.euclid.shape.primitives.Cylinder3D;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple3D.interfaces.Point3DBasics;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;
import us.ihmc.simulationconstructionset.physics.CollisionShapeDescription;

public class CylinderShapeDescription<T extends CylinderShapeDescription<T>> implements CollisionShapeDescription<T>
{
   private double smoothingRadius = 0.0;

   private final Cylinder3D cylinder3d;

   public CylinderShapeDescription(double radius, double height)
   {
      cylinder3d = new Cylinder3D(height, radius);
   }

   public double getRadius()
   {
      return cylinder3d.getRadius();
   }

   public double getHeight()
   {
      return cylinder3d.getLength();
   }

   public void set(CylinderShapeDescription<T> other)
   {
      smoothingRadius = other.smoothingRadius;
      cylinder3d.set(other.cylinder3d);
   }

   @Override
   public CylinderShapeDescription<T> copy()
   {
      CylinderShapeDescription<T> copy = new CylinderShapeDescription<>(cylinder3d.getRadius(), cylinder3d.getLength());
      copy.set(copy);
      return copy;
   }

   public double getSmoothingRadius()
   {
      return smoothingRadius;
   }

   @Override
   public void applyTransform(RigidBodyTransform transformToWorld)
   {
      cylinder3d.applyTransform(transformToWorld);
   }

   @Override
   public void setFrom(T cylinder)
   {
      cylinder3d.set(cylinder.getSupportingVertexHolder());
   }

   public Cylinder3D getSupportingVertexHolder()
   {
      return cylinder3d;
   }

   public void getProjection(Point3D pointToProject, Point3D closestPointOnCylinderToPack)
   {
      closestPointOnCylinderToPack.set(pointToProject);
      cylinder3d.orthogonalProjection(closestPointOnCylinderToPack);
   }

   @Override
   public void getBoundingBox(BoundingBox3D boundingBoxToPack)
   {
      boundingBoxToPack.set(cylinder3d.getBoundingBox());
   }

   @Override
   public boolean isPointInside(Point3D pointInWorld)
   {
      return cylinder3d.distance(pointInWorld) <= smoothingRadius;
   }

   @Override
   public boolean rollContactIfRolling(Vector3D surfaceNormal, Point3D pointToRoll)
   {
      return projectToBottomOfCurvedSurface(surfaceNormal, pointToRoll);
   }

   private final Vector3D orthogonalComponent = new Vector3D();

   /**
    * Projects the pointToProject to the bottom of a curved surface given the surfaceNormal defining
    * how the curved surface is contacting another surface, as when the two surfaces roll on each
    * other.
    *
    * @param surfaceNormal  defining another surface in which this object is in rolling contact on the
    *                       other surface.
    * @param pointToProject point to project to the bottom of the curved surface.
    * @return true if the point was projected. Otherwise false.
    */
   public boolean projectToBottomOfCurvedSurface(Vector3DReadOnly surfaceNormal, Point3DBasics pointToProject)
   {
      double normalDot = surfaceNormal.dot(cylinder3d.getAxis());
      orthogonalComponent.scaleAdd(-normalDot, cylinder3d.getAxis(), surfaceNormal);

      boolean wasRolling = Math.abs(orthogonalComponent.getX()) >= 1e-7 || Math.abs(orthogonalComponent.getY()) >= 1e-7;

      orthogonalComponent.normalize();

      double positionAlongAxis = EuclidGeometryTools.percentageAlongLine3D(pointToProject, cylinder3d.getPosition(), cylinder3d.getAxis());
      pointToProject.set(cylinder3d.getPosition());
      pointToProject.scaleAdd(cylinder3d.getRadius(), orthogonalComponent, pointToProject);
      pointToProject.scaleAdd(positionAlongAxis, cylinder3d.getAxis(), pointToProject);

      return wasRolling;
   }
}
