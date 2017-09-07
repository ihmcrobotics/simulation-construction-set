package us.ihmc.simulationconstructionset.physics.collision.simple;

import us.ihmc.euclid.geometry.BoundingBox3D;
import us.ihmc.euclid.geometry.Cylinder3D;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple2D.Vector2D;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple3D.interfaces.Point3DBasics;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;
import us.ihmc.geometry.polytope.CylinderSupportingVertexHolder;
import us.ihmc.geometry.polytope.SupportingVertexHolder;
import us.ihmc.simulationconstructionset.physics.CollisionShapeDescription;

public class CylinderShapeDescription<T extends CylinderShapeDescription<T>> implements CollisionShapeDescription<T>
{
   private final CylinderSupportingVertexHolder supportingVertexHolder;
   private double radius;
   private double height;
   private double smoothingRadius = 0.0;

   private final RigidBodyTransform transform = new RigidBodyTransform();

   private final RigidBodyTransform cylinderConsistencyTransform = new RigidBodyTransform();

   private final BoundingBox3D boundingBox = new BoundingBox3D(Double.NEGATIVE_INFINITY, Double.NEGATIVE_INFINITY, Double.NEGATIVE_INFINITY, Double.POSITIVE_INFINITY, Double.POSITIVE_INFINITY, Double.POSITIVE_INFINITY);
   private boolean boundingBoxNeedsUpdating = true;

   //TODO: Get rid of this redundancy. Make cylinder definitions consistent...
   private final Cylinder3D cylinder3d;

   public CylinderShapeDescription(double radius, double height)
   {
      supportingVertexHolder = new CylinderSupportingVertexHolder(radius, height);

      this.radius = radius;
      this.height = height;

      cylinder3d = new Cylinder3D(height, radius);

      cylinderConsistencyTransform.setTranslation(0.0, 0.0, -height / 2.0);
      cylinder3d.setPose(cylinderConsistencyTransform);
      
      boundingBoxNeedsUpdating = true;
   }

   public double getRadius()
   {
      return radius;
   }

   public double getHeight()
   {
      return height;
   }

   public void getTransform(RigidBodyTransform transformToPack)
   {
      transformToPack.set(transform);
   }

   @Override
   public CylinderShapeDescription<T> copy()
   {
      CylinderShapeDescription<T> copy = new CylinderShapeDescription<T>(radius, height);
      copy.smoothingRadius = this.smoothingRadius;
      copy.setTransform(this.transform);
      return copy;
   }

   public void setTransform(RigidBodyTransform transform)
   {
      this.transform.set(transform);
      setSupportingVertexAndCylinder3dTransformFromThisAndConsistencyTransform();
   }

   private void setSupportingVertexAndCylinder3dTransformFromThisAndConsistencyTransform()
   {
      supportingVertexHolder.setTransform(transform);
      this.cylinder3d.setPose(this.transform);
      this.cylinder3d.appendTransform(cylinderConsistencyTransform);
      boundingBoxNeedsUpdating = true;
   }

   public double getSmoothingRadius()
   {
      return smoothingRadius;
   }

   @Override
   public void applyTransform(RigidBodyTransform transformToWorld)
   {
      transform.preMultiply(transformToWorld);
      setSupportingVertexAndCylinder3dTransformFromThisAndConsistencyTransform();
   }

   @Override
   public void setFrom(T cylinder)
   {
      this.radius = cylinder.getRadius();
      this.height = cylinder.getHeight();
      cylinderConsistencyTransform.setTranslation(0.0, 0.0, -height / 2.0);

      cylinder.getTransform(this.transform);

      setSupportingVertexAndCylinder3dTransformFromThisAndConsistencyTransform();
   }

   public SupportingVertexHolder getSupportingVertexHolder()
   {
      return supportingVertexHolder;
   }

   public void getProjection(Point3D pointToProject, Point3D closestPointOnCylinderToPack)
   {
      closestPointOnCylinderToPack.set(pointToProject);
      cylinder3d.orthogonalProjection(closestPointOnCylinderToPack);
   }

   @Override
   public void getBoundingBox(BoundingBox3D boundingBoxToPack)
   {
      if (boundingBoxNeedsUpdating)
      {
         updateBoundingBox();
         boundingBoxNeedsUpdating = false;
      }
      boundingBoxToPack.set(boundingBox);
   }

   private final Vector3D supportDirectionForBoundingBox = new Vector3D();

   private void updateBoundingBox()
   {
      supportDirectionForBoundingBox.set(1.0, 0.0, 0.0);
      Point3D supportingVertex = supportingVertexHolder.getSupportingVertex(supportDirectionForBoundingBox);
      double xMax = supportingVertex.getX();
      
      supportDirectionForBoundingBox.set(-1.0, 0.0, 0.0);
      supportingVertex = supportingVertexHolder.getSupportingVertex(supportDirectionForBoundingBox);
      double xMin = supportingVertex.getX();
      
      supportDirectionForBoundingBox.set(0.0, 1.0, 0.0);
      supportingVertex = supportingVertexHolder.getSupportingVertex(supportDirectionForBoundingBox);
      double yMax = supportingVertex.getY();
      
      supportDirectionForBoundingBox.set(0.0, -1.0, 0.0);
      supportingVertex = supportingVertexHolder.getSupportingVertex(supportDirectionForBoundingBox);
      double yMin = supportingVertex.getY();
      
      supportDirectionForBoundingBox.set(0.0, 0.0, 1.0);
      supportingVertex = supportingVertexHolder.getSupportingVertex(supportDirectionForBoundingBox);
      double zMax = supportingVertex.getZ();
      
      supportDirectionForBoundingBox.set(0.0, 0.0, -1.0);
      supportingVertex = supportingVertexHolder.getSupportingVertex(supportDirectionForBoundingBox);
      double zMin = supportingVertex.getZ();
      
      boundingBox.set(xMin, yMin, zMin, xMax, yMax, zMax);
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

   private final Point3D tempPointInWorldForProjectingToBottom = new Point3D();
   private final Vector3D tempVectorInWorldForProjectingToBottom = new Vector3D();

   /**
    * Projects the pointToProject to the bottom of a curved surface given the surfaceNormal defining
    * how the curved surface is contacting another surface, as when the two surfaces roll on each
    * other.
    *
    * @param surfaceNormal defining another surface in which this object is in rolling contact on
    *           the other surface.
    * @param pointToProject point to project to the bottom of the curved surface.
    * @return true if the point was projected. Otherwise false.
    */
   public boolean projectToBottomOfCurvedSurface(Vector3DReadOnly surfaceNormal, Point3DBasics pointToProject)
   {
      //TODO: Should this method be in Shape3d, or not even be here at all? Not sure...
      tempPointInWorldForProjectingToBottom.set(pointToProject);
      tempVectorInWorldForProjectingToBottom.set(surfaceNormal);
      cylinder3d.transformToLocal(tempPointInWorldForProjectingToBottom);
      cylinder3d.transformToLocal(tempVectorInWorldForProjectingToBottom);

      boolean wasRolling = projectToBottomOfCurvedSurfaceInShapeFrame(tempVectorInWorldForProjectingToBottom, tempPointInWorldForProjectingToBottom);
      cylinder3d.transformToWorld(tempPointInWorldForProjectingToBottom);

      pointToProject.set(tempPointInWorldForProjectingToBottom);
      return wasRolling;
   }

   private final Vector2D tempVectorInShapeFrameForProjectingToBottom = new Vector2D();

   private boolean projectToBottomOfCurvedSurfaceInShapeFrame(Vector3DReadOnly normalInShapeFrame, Point3DBasics pointToRollInShapeFrame)
   {
      double x = normalInShapeFrame.getX();
      double y = normalInShapeFrame.getY();

      if (Math.abs(x) < 1e-7 && Math.abs(y) < 1e-7)
         return false;

      tempVectorInShapeFrameForProjectingToBottom.set(x, normalInShapeFrame.getY());
      tempVectorInShapeFrameForProjectingToBottom.normalize();
      tempVectorInShapeFrameForProjectingToBottom.scale(radius);

      pointToRollInShapeFrame.set(tempVectorInShapeFrameForProjectingToBottom.getX(), tempVectorInShapeFrameForProjectingToBottom.getY(),
                                  pointToRollInShapeFrame.getZ());
      return true;
   }
}
