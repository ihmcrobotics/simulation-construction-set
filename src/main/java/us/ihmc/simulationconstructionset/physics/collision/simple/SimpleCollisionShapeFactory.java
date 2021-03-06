package us.ihmc.simulationconstructionset.physics.collision.simple;

import java.util.ArrayList;
import java.util.List;

import us.ihmc.euclid.geometry.LineSegment3D;
import us.ihmc.euclid.geometry.interfaces.LineSegment3DReadOnly;
import us.ihmc.euclid.geometry.interfaces.Vertex3DSupplier;
import us.ihmc.euclid.shape.convexPolytope.ConvexPolytope3D;
import us.ihmc.euclid.shape.convexPolytope.tools.EuclidPolytopeFactories;
import us.ihmc.euclid.shape.primitives.Box3D;
import us.ihmc.euclid.shape.primitives.Capsule3D;
import us.ihmc.euclid.shape.primitives.Cylinder3D;
import us.ihmc.euclid.shape.primitives.Ramp3D;
import us.ihmc.euclid.shape.primitives.Sphere3D;
import us.ihmc.euclid.shape.primitives.interfaces.Shape3DReadOnly;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.robotics.robotDescription.CapsuleDescriptionReadOnly;
import us.ihmc.robotics.robotDescription.CollisionMeshDescription;
import us.ihmc.robotics.robotDescription.ConvexShapeDescriptionReadOnly;
import us.ihmc.robotics.robotDescription.CubeDescriptionReadOnly;
import us.ihmc.robotics.robotDescription.CylinderDescriptionReadOnly;
import us.ihmc.robotics.robotDescription.SphereDescriptionReadOnly;
import us.ihmc.simulationconstructionset.Link;
import us.ihmc.simulationconstructionset.physics.CollisionShape;
import us.ihmc.simulationconstructionset.physics.CollisionShapeDescription;
import us.ihmc.simulationconstructionset.physics.CollisionShapeFactory;
import us.ihmc.simulationconstructionset.physics.SimpleCollisionShapeWithLink;

public class SimpleCollisionShapeFactory implements CollisionShapeFactory
{
   private final SimpleCollisionDetector detector;

   public SimpleCollisionShapeFactory(SimpleCollisionDetector detector)
   {
      this.detector = detector;
   }

   @Override
   public void setMargin(double margin)
   {
   }

   @Override
   public CollisionShapeDescription<?> createSimpleCollisionShape(Shape3DReadOnly shape3D)
   {
      if ((shape3D instanceof Box3D))
         return createBox(shape3D);
      if ((shape3D instanceof Sphere3D))
         return createSphere(shape3D);
      if ((shape3D instanceof Cylinder3D))
         return createCylinder(shape3D);
      if ((shape3D instanceof Capsule3D))
         return createCapsule(shape3D);
      if ((shape3D instanceof Ramp3D))
         return createRamp(shape3D);

      throw new IllegalArgumentException("The type of " + shape3D.getClass() + " is not matched among the simple shape Box3D, Sphere3D, Cylinder3D, Capsule3D");
   }

   private CollisionShapeDescription<?> createBox(Shape3DReadOnly shape3D)
   {
      if (!(shape3D instanceof Box3D))
         throw new IllegalArgumentException("Check Shape3D is Box3D");
      Box3D box3D = (Box3D) shape3D;
      return createBox(0.5 * box3D.getSizeX(), 0.5 * box3D.getSizeY(), 0.5 * box3D.getSizeZ());
   }

   private CollisionShapeDescription<?> createSphere(Shape3DReadOnly shape3D)
   {
      if (!(shape3D instanceof Sphere3D))
         throw new IllegalArgumentException("Check Shape3D is Sphere3D");
      Sphere3D sphere3D = (Sphere3D) shape3D;
      return createSphere(sphere3D.getRadius());
   }

   private CollisionShapeDescription<?> createCylinder(Shape3DReadOnly shape3D)
   {
      if (!(shape3D instanceof Cylinder3D))
         throw new IllegalArgumentException("Check Shape3D is Cylinder3D");
      Cylinder3D cylinder3D = (Cylinder3D) shape3D;
      return createCylinder(cylinder3D.getRadius(), cylinder3D.getLength());
   }

   private CollisionShapeDescription<?> createCapsule(Shape3DReadOnly shape3D)
   {
      if (!(shape3D instanceof Capsule3D))
         throw new IllegalArgumentException("Check Shape3D is Capsule3D");
      Capsule3D capsule3D = (Capsule3D) shape3D;
      return createCapsule(capsule3D.getRadius(), capsule3D.getLength());
   }

   private CollisionShapeDescription<?> createRamp(Shape3DReadOnly shape3D)
   {
      if (!(shape3D instanceof Ramp3D))
         throw new IllegalArgumentException("Check Shape3D is Ramp3D");
      Ramp3D ramp3D = (Ramp3D) shape3D;
      ConvexPolytope3D polytope = new ConvexPolytope3D(Vertex3DSupplier.asVertex3DSupplier(ramp3D.getVertices()));
      return new PolytopeShapeDescription<>(polytope);
   }

   @Override
   public CollisionShapeDescription<?> createBox(double halfLengthX, double halfWidthY, double halfHeightZ)
   {
      List<Point3D> cubeVertices = EuclidPolytopeFactories.newCubeVertices(2.0);
      cubeVertices.forEach(vertex -> vertex.scale(halfLengthX, halfWidthY, halfHeightZ));
      ConvexPolytope3D polytope = new ConvexPolytope3D(Vertex3DSupplier.asVertex3DSupplier(cubeVertices));
      return new PolytopeShapeDescription<>(polytope);
   }

   @Override
   public CollisionShapeDescription<?> createCylinder(double radius, double height)
   {
      return new CylinderShapeDescription<>(radius, height);
   }

   @Override
   public CollisionShapeDescription<?> createSphere(double radius)
   {
      return new SphereShapeDescription<>(radius, new Point3D());
   }

   @Override
   public CollisionShapeDescription<?> createCapsule(double radius, double height)
   {
      return new CapsuleShapeDescription<>(radius, height);
   }

   public CollisionShapeDescription<?> createCapsule(double radius, LineSegment3DReadOnly capToCapLineSegment)
   {
      return new CapsuleShapeDescription<>(radius, capToCapLineSegment);
   }

   @Override
   public CollisionShape addShape(Link link, RigidBodyTransform shapeToLink, CollisionShapeDescription<?> description, boolean isGround, int groupMask,
                                  int collisionMask)
   {
      SimpleCollisionShapeWithLink collisionShape = new SimpleCollisionShapeWithLink(link, description, shapeToLink);
      collisionShape.setIsGround(isGround);
      collisionShape.setCollisionGroup(groupMask);
      collisionShape.setCollisionMask(collisionMask);
      detector.addShape(collisionShape);

      return collisionShape;
   }

   @Override
   public CollisionShape addShape(CollisionShapeDescription<?> description)
   {
      Link link = null;
      RigidBodyTransform shapeToLink = new RigidBodyTransform();
      boolean isGround = false;
      int collisionGroup = 0xFFFF;
      int collisionMask = 0xFFFF;

      return addShape(link, shapeToLink, description, isGround, collisionGroup, collisionMask);
   }

   @Override
   public void addCollisionMeshDescription(Link link, CollisionMeshDescription collisionMeshDescription)
   {
      ArrayList<ConvexShapeDescriptionReadOnly> convexShapeDescriptions = new ArrayList<>();
      collisionMeshDescription.getConvexShapeDescriptions(convexShapeDescriptions);

      for (ConvexShapeDescriptionReadOnly convexShapeDescription : convexShapeDescriptions)
      {
         if (convexShapeDescription instanceof CapsuleDescriptionReadOnly)
         {
            CapsuleDescriptionReadOnly capsule = (CapsuleDescriptionReadOnly) convexShapeDescription;

            LineSegment3D capToCapLineSegment = new LineSegment3D();
            capsule.getCapToCapLineSegment(capToCapLineSegment);

            CollisionShapeDescription<?> collisionShapeDescription = createCapsule(capsule.getRadius(), capToCapLineSegment);
            addShape(link,
                     null,
                     collisionShapeDescription,
                     collisionMeshDescription.getIsGround(),
                     collisionMeshDescription.getCollisionGroup(),
                     collisionMeshDescription.getCollisionMask());
         }

         else if (convexShapeDescription instanceof SphereDescriptionReadOnly)
         {
            SphereDescriptionReadOnly sphere = (SphereDescriptionReadOnly) convexShapeDescription;
            CollisionShapeDescription<?> collisionShapeDescription = createSphere(sphere.getRadius());
            RigidBodyTransform transform = new RigidBodyTransform();
            sphere.getRigidBodyTransform(transform);
            addShape(link,
                     transform,
                     collisionShapeDescription,
                     collisionMeshDescription.getIsGround(),
                     collisionMeshDescription.getCollisionGroup(),
                     collisionMeshDescription.getCollisionMask());
         }

         else if (convexShapeDescription instanceof CubeDescriptionReadOnly)
         {
            CubeDescriptionReadOnly cube = (CubeDescriptionReadOnly) convexShapeDescription;
            CollisionShapeDescription<?> collisionShapeDescription = createBox(cube.getLengthX() / 2.0, cube.getWidthY() / 2.0, cube.getHeightZ() / 2.0);
            RigidBodyTransform transform = new RigidBodyTransform();
            cube.getRigidBodyTransformToCenter(transform);
            addShape(link,
                     transform,
                     collisionShapeDescription,
                     collisionMeshDescription.getIsGround(),
                     collisionMeshDescription.getCollisionGroup(),
                     collisionMeshDescription.getCollisionMask());
         }

         else if (convexShapeDescription instanceof CylinderDescriptionReadOnly)
         {
            CylinderDescriptionReadOnly cylinder = (CylinderDescriptionReadOnly) convexShapeDescription;
            CollisionShapeDescription<?> collisionShapeDescription = createCylinder(cylinder.getRadius(), cylinder.getHeight());
            RigidBodyTransform transform = new RigidBodyTransform();
            cylinder.getRigidBodyTransformToCenter(transform);
            addShape(link,
                     transform,
                     collisionShapeDescription,
                     collisionMeshDescription.getIsGround(),
                     collisionMeshDescription.getCollisionGroup(),
                     collisionMeshDescription.getCollisionMask());
         }

         else
         {
            throw new IllegalArgumentException(getClass().getSimpleName() + ". Don't recognize convexShapeDescription type!");
         }
      }

   }

}
