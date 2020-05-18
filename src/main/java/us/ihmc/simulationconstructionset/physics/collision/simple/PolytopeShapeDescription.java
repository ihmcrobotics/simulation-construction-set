package us.ihmc.simulationconstructionset.physics.collision.simple;

import us.ihmc.euclid.geometry.BoundingBox3D;
import us.ihmc.euclid.shape.convexPolytope.ConvexPolytope3D;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.simulationconstructionset.physics.CollisionShapeDescription;

public class PolytopeShapeDescription<T extends PolytopeShapeDescription<T>> implements CollisionShapeDescription<T>
{
   private final ConvexPolytope3D polytope;
   private double smoothingRadius = 0.0;

   public PolytopeShapeDescription(ConvexPolytope3D polytope)
   {
      this.polytope = polytope;
   }

   @Override
   public PolytopeShapeDescription<T> copy()
   {
      ConvexPolytope3D polytopeCopy = new ConvexPolytope3D(polytope);
      PolytopeShapeDescription<T> copy = new PolytopeShapeDescription<>(polytopeCopy);
      copy.setSmoothingRadius(smoothingRadius);
      return copy;
   }

   public ConvexPolytope3D getPolytope()
   {
      return polytope;
   }

   @Override
   public void setFrom(T polytopeShapeDescription)
   {
      this.polytope.set(polytopeShapeDescription.getPolytope());
   }

   @Override
   public void applyTransform(RigidBodyTransform transform)
   {
      this.polytope.applyTransform(transform);
   }

   public void setSmoothingRadius(double smoothingRadius)
   {
      this.smoothingRadius = smoothingRadius;
   }

   public double getSmoothingRadius()
   {
      return smoothingRadius;
   }

   @Override
   public void getBoundingBox(BoundingBox3D boundingBoxToPack)
   {
      polytope.getBoundingBox(boundingBoxToPack);
   }

   @Override
   public boolean isPointInside(Point3D pointInWorld)
   {
      return polytope.isPointInside(pointInWorld);
   }

   @Override
   public boolean rollContactIfRolling(Vector3D surfaceNormal, Point3D pointToRoll)
   {
      if (smoothingRadius != 0.0)
      {
         throw new RuntimeException("Implement me for nonzero smoothing radius!");
      }

      return false;
   }

}
