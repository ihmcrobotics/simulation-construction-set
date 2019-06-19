package us.ihmc.simulationconstructionset.physics.collision.simple;

import java.util.ArrayList;
import java.util.Random;

import us.ihmc.euclid.geometry.BoundingBox3D;
import us.ihmc.euclid.geometry.LineSegment3D;
import us.ihmc.euclid.shape.collision.EuclidShape3DCollisionResult;
import us.ihmc.euclid.shape.collision.epa.ExpandingPolytopeAlgorithm;
import us.ihmc.euclid.shape.collision.gjk.GilbertJohnsonKeerthiCollisionDetector;
import us.ihmc.euclid.shape.collision.interfaces.SupportingVertexHolder;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple3D.interfaces.Point3DBasics;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;
import us.ihmc.simulationconstructionset.physics.CollisionShape;
import us.ihmc.simulationconstructionset.physics.CollisionShapeDescription;
import us.ihmc.simulationconstructionset.physics.CollisionShapeFactory;
import us.ihmc.simulationconstructionset.physics.ScsCollisionDetector;
import us.ihmc.simulationconstructionset.physics.collision.CollisionDetectionResult;

public class SimpleCollisionDetector implements ScsCollisionDetector
{
   private boolean VERBOSE = false;

   private final ArrayList<CollisionShape> collisionObjects = new ArrayList<CollisionShape>();

   // Temporary variables for computation:
   private final Point3D centerOne = new Point3D();
   private final Point3D centerTwo = new Point3D();
   private final Vector3D tempVector = new Vector3D();

   public SimpleCollisionDetector()
   {
   }

   @Override
   public void initialize()
   {
   }

   @Override
   public CollisionShapeFactory getShapeFactory()
   {
      return new SimpleCollisionShapeFactory(this);
   }

   private final Random random = new Random(1776L);

   //   private final THashMap<CollisionShapeFactory, ArrayList<CollisionShape>> collidingPairs = new THashMap<>();
   private boolean[][] haveCollided = null;

   private boolean useSimpleSpeedupMethod = false;

   //TODO: Get rid of the need for this by first phase collision detection.
   private double percentChanceCheckCollision = 0.9;


   public void setUseSimpleSpeedupMethod()
   {
      this.useSimpleSpeedupMethod = true;
   }

   private final BoundingBox3D boundingBoxOne = new BoundingBox3D(0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
   private final BoundingBox3D boundingBoxTwo = new BoundingBox3D(0.0, 0.0, 0.0, 0.0, 0.0, 0.0);

   @Override
   public void performCollisionDetection(CollisionDetectionResult result)
   {  
      int boundingBoxChecks = 0;
      int collisionChecks = 0;
      int numberOfCollisions = 0;
      
      int numberOfObjects = collisionObjects.size();

      if (useSimpleSpeedupMethod && (haveCollided == null))
      {
         haveCollided = new boolean[numberOfObjects][numberOfObjects];
      }

      for (int i = 0; i < numberOfObjects; i++)
      {
         CollisionShape collisionShape = collisionObjects.get(i);
         collisionShape.computeTransformedCollisionShape();
      }

      for (int i = 0; i < numberOfObjects; i++)
      {
         CollisionShape objectOne = collisionObjects.get(i);
         CollisionShapeDescription<?> descriptionOne = objectOne.getTransformedCollisionShapeDescription();

         for (int j = i + 1; j < numberOfObjects; j++)
         {
            if ((useSimpleSpeedupMethod) && (!haveCollided[i][j]) && (random.nextDouble() < percentChanceCheckCollision ))
               continue;

            CollisionShape objectTwo = collisionObjects.get(j);

            if(objectOne.isGround() && objectTwo.isGround())
               continue;

            CollisionShapeDescription<?> descriptionTwo = objectTwo.getTransformedCollisionShapeDescription();
            
            if ((objectOne.getCollisionGroup() & objectTwo.getCollisionMask()) == 0x00)
            {
               continue;
            }
            
            if ((objectTwo.getCollisionGroup() & objectOne.getCollisionMask()) == 0x00)
            {
               continue;
            }
            
            objectOne.getBoundingBox(boundingBoxOne);
            objectTwo.getBoundingBox(boundingBoxTwo);

            boundingBoxChecks++;
            if (!boundingBoxOne.intersectsInclusive(boundingBoxTwo))
            {
               continue;
            }
            
            collisionChecks++;
            boolean areColliding = false;

            //TODO: Make this shorter and more efficient...
            //TODO: Add Plane
            if ((descriptionOne instanceof SphereShapeDescription) && (descriptionTwo instanceof SphereShapeDescription))
            {
               areColliding = doSphereSphereCollisionDetection(objectOne, (SphereShapeDescription<?>) descriptionOne, objectTwo,
                     (SphereShapeDescription<?>) descriptionTwo, result);
            }
            else if ((descriptionOne instanceof CapsuleShapeDescription) && (descriptionTwo instanceof CapsuleShapeDescription))
            {
               areColliding = doCapsuleCapsuleCollisionDetection(objectOne, (CapsuleShapeDescription<?>) descriptionOne, objectTwo,
                     (CapsuleShapeDescription<?>) descriptionTwo, result);
            }
            else if ((descriptionOne instanceof PolytopeShapeDescription) && (descriptionTwo instanceof PolytopeShapeDescription))
            {
               areColliding = doPolytopePolytopeCollisionDetection(objectOne, (PolytopeShapeDescription<?>) descriptionOne, objectTwo,
                     (PolytopeShapeDescription<?>) descriptionTwo, result);
            }
            else if ((descriptionOne instanceof CylinderShapeDescription) && (descriptionTwo instanceof CylinderShapeDescription))
            {
               areColliding = doCylinderCylinderCollisionDetection(objectOne, (CylinderShapeDescription<?>) descriptionOne, objectTwo,
                     (CylinderShapeDescription<?>) descriptionTwo, result);
            }


            else if ((descriptionOne instanceof SphereShapeDescription) && (descriptionTwo instanceof CapsuleShapeDescription))
            {
               areColliding = doCapsuleSphereCollisionDetection(objectTwo, (CapsuleShapeDescription<?>) descriptionTwo, objectOne,
                     (SphereShapeDescription<?>) descriptionOne, result);
            }
            else if ((descriptionOne instanceof CapsuleShapeDescription) && (descriptionTwo instanceof SphereShapeDescription))
            {
               areColliding = doCapsuleSphereCollisionDetection(objectOne, (CapsuleShapeDescription<?>) descriptionOne, objectTwo,
                     (SphereShapeDescription<?>) descriptionTwo, result);
            }

            else if ((descriptionOne instanceof SphereShapeDescription) && (descriptionTwo instanceof PolytopeShapeDescription))
            {
               areColliding = doSpherePolytopeCollisionDetection(objectOne, (SphereShapeDescription<?>) descriptionOne, objectTwo,
                     (PolytopeShapeDescription<?>) descriptionTwo, result);
            }
            else if ((descriptionOne instanceof PolytopeShapeDescription) && (descriptionTwo instanceof SphereShapeDescription))
            {
               areColliding = doSpherePolytopeCollisionDetection(objectTwo, (SphereShapeDescription<?>) descriptionTwo, objectOne,
                     (PolytopeShapeDescription<?>) descriptionOne, result);
            }

            else if ((descriptionOne instanceof SphereShapeDescription) && (descriptionTwo instanceof CylinderShapeDescription))
            {
               areColliding = doSphereCylinderCollisionDetection(objectOne, (SphereShapeDescription<?>) descriptionOne, objectTwo,
                     (CylinderShapeDescription<?>) descriptionTwo, result);
            }
            else if ((descriptionOne instanceof CylinderShapeDescription) && (descriptionTwo instanceof SphereShapeDescription))
            {
               areColliding = doSphereCylinderCollisionDetection(objectTwo, (SphereShapeDescription<?>) descriptionTwo, objectOne,
                     (CylinderShapeDescription<?>) descriptionOne, result);
            }


            else if ((descriptionOne instanceof CapsuleShapeDescription) && (descriptionTwo instanceof PolytopeShapeDescription))
            {
               areColliding = doCapsulePolytopeCollisionDetection(objectOne, (CapsuleShapeDescription<?>) descriptionOne, objectTwo,
                     (PolytopeShapeDescription<?>) descriptionTwo, result);
            }
            else if ((descriptionOne instanceof PolytopeShapeDescription) && (descriptionTwo instanceof CapsuleShapeDescription))
            {
               areColliding = doCapsulePolytopeCollisionDetection(objectTwo, (CapsuleShapeDescription<?>) descriptionTwo, objectOne,
                     (PolytopeShapeDescription<?>) descriptionOne, result);
            }
            else if ((descriptionOne instanceof CapsuleShapeDescription) && (descriptionTwo instanceof CylinderShapeDescription))
            {
               areColliding = doCapsuleCylinderCollisionDetection(objectOne, (CapsuleShapeDescription<?>) descriptionOne, objectTwo,
                     (CylinderShapeDescription<?>) descriptionTwo, result);
            }
            else if ((descriptionOne instanceof CylinderShapeDescription) && (descriptionTwo instanceof CapsuleShapeDescription))
            {
               areColliding = doCapsuleCylinderCollisionDetection(objectTwo, (CapsuleShapeDescription<?>) descriptionTwo, objectOne,
                     (CylinderShapeDescription<?>) descriptionOne, result);
            }

            else if ((descriptionOne instanceof PolytopeShapeDescription) && (descriptionTwo instanceof CylinderShapeDescription))
            {
               areColliding = doCylinderPolytopeCollisionDetection(objectTwo, (CylinderShapeDescription<?>) descriptionTwo, objectOne,
                     (PolytopeShapeDescription<?>) descriptionOne, result);
            }
            else if ((descriptionOne instanceof CylinderShapeDescription) && (descriptionTwo instanceof PolytopeShapeDescription))
            {
               areColliding = doCylinderPolytopeCollisionDetection(objectOne, (CylinderShapeDescription<?>) descriptionOne, objectTwo,
                     (PolytopeShapeDescription<?>) descriptionTwo, result);
            }

            else if ((descriptionOne instanceof BoxShapeDescription) && (descriptionTwo instanceof BoxShapeDescription))
            {
               areColliding = doBoxBoxCollisionDetection(objectOne, (BoxShapeDescription<?>) descriptionOne, objectTwo, (BoxShapeDescription<?>) descriptionTwo,
                     result);
            }

            if (areColliding)
            {
               numberOfCollisions++;

               if (useSimpleSpeedupMethod) haveCollided[i][j] = true;
               //               ArrayList<CollisionShape> arrayList = collidingPairs.get(objectOne);
               //               if (arrayList == null)
               //               {
               //                  arrayList = new ArrayList<>();
               //                  collidingPairs.put(objectOne, arrayList);
               //               }
               //
               //               if (!arrayList.contains(objectTwo))
               //               {
               //                  arrayList.add(objectTwo);
               //               }
            }
         }
      }
      
      if (VERBOSE)
      {
         System.out.println("\nboundingBoxChecks = " + boundingBoxChecks);
         System.out.println("collisionChecks = " + collisionChecks);
         System.out.println("numberOfCollisions = " + numberOfCollisions);
      }
   }

   public ArrayList<CollisionShape> getCollisionObjects()
   {
      return collisionObjects;
   }

   private boolean doPolytopePolytopeCollisionDetection(CollisionShape objectOne, PolytopeShapeDescription<?> polytopeShapeDescriptionOne,
         CollisionShape objectTwo, PolytopeShapeDescription<?> polytopeShapeDescriptionTwo, CollisionDetectionResult result)
   {
      SupportingVertexHolder polytopeOne = polytopeShapeDescriptionOne.getPolytope();
      SupportingVertexHolder polytopeTwo = polytopeShapeDescriptionTwo.getPolytope();

      double radiusOne = polytopeShapeDescriptionOne.getSmoothingRadius();
      double radiusTwo = polytopeShapeDescriptionTwo.getSmoothingRadius();

      return doPolytopePolytopeCollisionDetection(objectOne, polytopeOne, radiusOne, objectTwo, polytopeTwo, radiusTwo, result);
   }

   private boolean doCylinderPolytopeCollisionDetection(CollisionShape objectOne, CylinderShapeDescription<?> descriptionOne, CollisionShape objectTwo,
         PolytopeShapeDescription<?> descriptionTwo, CollisionDetectionResult result)
   {
      SupportingVertexHolder polytopeOne = descriptionOne.getSupportingVertexHolder();
      SupportingVertexHolder polytopeTwo = descriptionTwo.getPolytope();

      double radiusOne = descriptionOne.getSmoothingRadius();
      double radiusTwo = descriptionTwo.getSmoothingRadius();

      return doPolytopePolytopeCollisionDetection(objectOne, polytopeOne, radiusOne, objectTwo, polytopeTwo, radiusTwo, result);
   }

   private boolean doCylinderCylinderCollisionDetection(CollisionShape objectOne, CylinderShapeDescription<?> descriptionOne, CollisionShape objectTwo,
         CylinderShapeDescription<?> descriptionTwo, CollisionDetectionResult result)
   {
      SupportingVertexHolder polytopeOne = descriptionOne.getSupportingVertexHolder();
      SupportingVertexHolder polytopeTwo = descriptionTwo.getSupportingVertexHolder();

      double radiusOne = descriptionOne.getSmoothingRadius();
      double radiusTwo = descriptionTwo.getSmoothingRadius();

      return doPolytopePolytopeCollisionDetection(objectOne, polytopeOne, radiusOne, objectTwo, polytopeTwo, radiusTwo, result);
   }

   private boolean doBoxBoxCollisionDetection(CollisionShape objectOne, BoxShapeDescription<?> descriptionOne, CollisionShape objectTwo,
         BoxShapeDescription<?> descriptionTwo, CollisionDetectionResult result)
   {
      return false;
   }

   private final LineSegment3D lineSegmentOne = new LineSegment3D();
   private final LineSegment3D lineSegmentTwo = new LineSegment3D();
   private final Point3D closestPointOnOne = new Point3D();
   private final Point3D closestPointOnTwo = new Point3D();

   private boolean doCapsuleSphereCollisionDetection(CollisionShape objectOne, CapsuleShapeDescription<?> descriptionOne, CollisionShape objectTwo,
         SphereShapeDescription<?> descriptionTwo, CollisionDetectionResult result)
   {
      double capsuleRadius = descriptionOne.getRadius();
      descriptionOne.getLineSegment(lineSegmentOne);

      double sphereRadius = descriptionTwo.getRadius();
      descriptionTwo.getCenter(centerOfSphere);

      lineSegmentOne.orthogonalProjection(centerOfSphere, closestPointOnOne);

      double distanceSquared = centerOfSphere.distanceSquared(closestPointOnOne);

      if (distanceSquared <= (capsuleRadius + sphereRadius) * (capsuleRadius + sphereRadius))
      {
         addCollisionPairToResult(closestPointOnOne, centerOfSphere, capsuleRadius, sphereRadius, distanceSquared, objectOne, objectTwo, result);
         return true;
      }

      else
      {
         return false;
      }
   }

   private boolean doSphereCylinderCollisionDetection(CollisionShape objectOne, SphereShapeDescription<?> descriptionOne, CollisionShape objectTwo,
         CylinderShapeDescription<?> descriptionTwo, CollisionDetectionResult result)
   {
      double sphereRadius = descriptionOne.getRadius();
      descriptionOne.getCenter(centerOfSphere);

      descriptionTwo.getProjection(centerOfSphere, closestPointOnTwo);
      double cylinderSmoothingRadius = descriptionTwo.getSmoothingRadius();

      double distanceSquared = centerOfSphere.distanceSquared(closestPointOnTwo);

      if (distanceSquared <= (cylinderSmoothingRadius + sphereRadius) * (cylinderSmoothingRadius + sphereRadius))
      {
         addCollisionPairToResult(centerOfSphere, closestPointOnTwo, sphereRadius, cylinderSmoothingRadius, distanceSquared, objectOne, objectTwo, result);
         return true;
      }

      else
      {
         return false;
      }
   }

   private boolean doCapsuleCylinderCollisionDetection(CollisionShape capsuleShape, CapsuleShapeDescription<?> capsuleDescription, CollisionShape cylinderShape,
         CylinderShapeDescription<?> cylinderDescription, CollisionDetectionResult result)
   {
      double cylinderSmoothingRadius = cylinderDescription.getSmoothingRadius();
      SupportingVertexHolder cylinderSupportingVertexHolder = cylinderDescription.getSupportingVertexHolder();

      return doCapsuleSupportingVertexHolderCollisionDetection(capsuleShape, capsuleDescription, cylinderShape, cylinderSupportingVertexHolder, cylinderSmoothingRadius, result);
   }

   private boolean doCapsuleCapsuleCollisionDetection(CollisionShape objectOne, CapsuleShapeDescription<?> descriptionOne, CollisionShape objectTwo,
         CapsuleShapeDescription<?> descriptionTwo, CollisionDetectionResult result)
   {
      double radiusOne = descriptionOne.getRadius();
      double radiusTwo = descriptionTwo.getRadius();

      descriptionOne.getLineSegment(lineSegmentOne);
      descriptionTwo.getLineSegment(lineSegmentTwo);

      getClosestPointsOnLineSegments(lineSegmentOne, lineSegmentTwo, closestPointOnOne, closestPointOnTwo);

      double distanceSquared = closestPointOnOne.distanceSquared(closestPointOnTwo);

      if (distanceSquared <= (radiusOne + radiusTwo) * (radiusOne + radiusTwo))
      {
         addCollisionPairToResult(closestPointOnOne, closestPointOnTwo, radiusOne, radiusTwo, distanceSquared, objectOne, objectTwo, result);
         return true;
      }

      return false;
   }

   private void addCollisionPairToResult(Point3D pointOne, Point3D pointTwo, double radiusOne, double radiusTwo, double distanceSquared,
         CollisionShape objectOne, CollisionShape objectTwo, CollisionDetectionResult result)
   {
      Vector3D normalVector = new Vector3D();

      normalVector.sub(pointTwo, pointOne);

      // TODO: Get the normal from the features if the points are close.
      if (normalVector.lengthSquared() < 1e-10) return;
      normalVector.normalize();

      Point3D pointOnOne = new Point3D(pointOne);
      tempVector.set(normalVector);
      tempVector.scale(radiusOne);
      pointOnOne.add(tempVector);

      Point3D pointOnTwo = new Point3D(pointTwo);
      tempVector.set(normalVector);
      tempVector.scale(-radiusTwo);
      pointOnTwo.add(tempVector);

      double distance = Math.sqrt(distanceSquared) - radiusOne - radiusTwo;

      SimpleContactWrapper contacts = new SimpleContactWrapper(objectOne, objectTwo);
      contacts.addContact(pointOnOne, pointOnTwo, normalVector, distance);

      result.addContact(contacts);
   }

   private boolean doSphereSphereCollisionDetection(CollisionShape objectOne, SphereShapeDescription<?> descriptionOne, CollisionShape objectTwo,
         SphereShapeDescription<?> descriptionTwo, CollisionDetectionResult result)
   {
      double radiusOne = descriptionOne.getRadius();
      double radiusTwo = descriptionTwo.getRadius();

      descriptionOne.getCenter(centerOne);
      descriptionTwo.getCenter(centerTwo);

      double distanceSquared = centerOne.distanceSquared(centerTwo);

      if (distanceSquared <= (radiusOne + radiusTwo) * (radiusOne + radiusTwo))
      {
         addCollisionPairToResult(centerOne, centerTwo, radiusOne, radiusTwo, distanceSquared, objectOne, objectTwo, result);
         return true;
      }

      return false;
   }

   private final GilbertJohnsonKeerthiCollisionDetector gjkCollisionDetector = new GilbertJohnsonKeerthiCollisionDetector();
   private final ExpandingPolytopeAlgorithm expandingPolytopeAlgorithm = new ExpandingPolytopeAlgorithm();
   private final Point3D pointOnAToPack = new Point3D();
   private final Point3D pointOnBToPack = new Point3D();

   private final Point3D centerOfSphere = new Point3D();

   private boolean doSpherePolytopeCollisionDetection(CollisionShape objectOne, SphereShapeDescription<?> descriptionOne, CollisionShape objectTwo,
         PolytopeShapeDescription<?> descriptionTwo, CollisionDetectionResult result)
   {
      descriptionOne.getCenter(centerOfSphere);
      double sphereRadius = descriptionOne.getRadius();
      double polytopeSmoothingRadius = descriptionTwo.getSmoothingRadius();

      //TODO: Remove trash generation...
      SupportingVertexHolder sphereAsSupportingVertexHolder = new SupportingVertexHolder()
      {
         @Override
         public boolean getSupportingVertex(Vector3DReadOnly supportDirection, Point3DBasics supportingVertexToPack)
         {
            supportingVertexToPack.set(centerOfSphere);
            return true;
         }
      };

      return doPolytopePolytopeCollisionDetection(objectOne, sphereAsSupportingVertexHolder, sphereRadius, objectTwo, descriptionTwo.getPolytope(),
            polytopeSmoothingRadius, result);
   }

   private final LineSegment3D tempLineSegment = new LineSegment3D();
   private final Vector3D tempSegmentPointVector = new Vector3D();

   private boolean doCapsulePolytopeCollisionDetection(CollisionShape objectOne, CapsuleShapeDescription<?> descriptionOne, CollisionShape objectTwo,
         PolytopeShapeDescription<?> descriptionTwo, CollisionDetectionResult result)
   {
      double polytopeSmoothingRadius = descriptionTwo.getSmoothingRadius();

      return doCapsuleSupportingVertexHolderCollisionDetection(objectOne, descriptionOne, objectTwo, descriptionTwo.getPolytope(), polytopeSmoothingRadius, result);
   }

      private boolean doCapsuleSupportingVertexHolderCollisionDetection(CollisionShape objectOne, CapsuleShapeDescription<?> descriptionOne, CollisionShape objectTwo,
            SupportingVertexHolder descriptionTwo, double smoothingRadiusTwo, CollisionDetectionResult result)
      {
      descriptionOne.getLineSegment(tempLineSegment);
      final Point3DBasics tempSegmentPointOne = tempLineSegment.getFirstEndpoint();
      final Point3DBasics tempSegmentPointTwo = tempLineSegment.getSecondEndpoint();

      double capsuleRadius = descriptionOne.getRadius();

      //TODO: Recycle object...
      SupportingVertexHolder capsuleAsSupportingVertexHolder = new SupportingVertexHolder()
      {
         @Override
         public boolean getSupportingVertex(Vector3DReadOnly supportDirection, Point3DBasics supportingVertexToPack)
         {
            tempSegmentPointVector.set(tempSegmentPointOne);
            double dotOne = tempSegmentPointVector.dot(supportDirection);

            tempSegmentPointVector.set(tempSegmentPointTwo);
            double dotTwo = tempSegmentPointVector.dot(supportDirection);

            if (dotOne > dotTwo)
               supportingVertexToPack.set(tempSegmentPointOne);
            else
               supportingVertexToPack.set(tempSegmentPointTwo);
            return true;
         }
      };

      return doPolytopePolytopeCollisionDetection(objectOne, capsuleAsSupportingVertexHolder, capsuleRadius, objectTwo, descriptionTwo,
            smoothingRadiusTwo, result);
   }

   private boolean doPolytopePolytopeCollisionDetection(CollisionShape objectOne, SupportingVertexHolder supportingVertexHolderOne, double radiusOne,
         CollisionShape objectTwo, SupportingVertexHolder supportingVertexHolderTwo, double radiusTwo, CollisionDetectionResult result)
   {
      EuclidShape3DCollisionResult collisionResult = gjkCollisionDetector.evaluateCollision(supportingVertexHolderOne, supportingVertexHolderTwo);
      boolean areColliding = collisionResult.areShapesColliding();

      if (!areColliding)
      {
         pointOnAToPack.set(collisionResult.getPointOnA());
         pointOnBToPack.set(collisionResult.getPointOnB());
         double separationDistanceForContact = radiusOne + radiusTwo;

         double distanceSquared = pointOnAToPack.distanceSquared(pointOnBToPack);
         //         System.out.println(distanceSquared);

         if (distanceSquared < separationDistanceForContact * separationDistanceForContact)
         {
            //TODO: Find more than one point per object...

            SimpleContactWrapper contacts = new SimpleContactWrapper(objectOne, objectTwo);
            Vector3D normalVector = new Vector3D();
            normalVector.sub(pointOnBToPack, pointOnAToPack);

            //TODO: Magic distance number...
            if (normalVector.lengthSquared() > 1e-6)
            {
               normalVector.normalize();
               double distanceToReport = -pointOnAToPack.distance(pointOnBToPack); //0.001; //TODO: Do we even need this?

               Point3D contactOnA = new Point3D(normalVector);
               contactOnA.scaleAdd(radiusOne, pointOnAToPack);

               Point3D contactOnB = new Point3D(normalVector);
               contactOnB.scaleAdd(-radiusTwo, pointOnBToPack);

               contacts.addContact(contactOnA, contactOnB, normalVector, distanceToReport);
               result.addContact(contacts);
               return true;
            }
         }
      }
      else
      {
            expandingPolytopeAlgorithm.evaluateCollision(supportingVertexHolderOne, supportingVertexHolderTwo, gjkCollisionDetector.getSimplex().getVertices(), collisionResult);
            pointOnAToPack.set(collisionResult.getPointOnA());
            pointOnBToPack.set(collisionResult.getPointOnB());

            if (!collisionResult.areShapesColliding())
               return false;

            //TODO: Reduce trash here...
            Vector3D collisionNormal = new Vector3D(expandingPolytopeAlgorithm.getClosestFace().getClosestPointToOrigin());

            //TODO: Magic number for normalize
            if (collisionNormal.lengthSquared() > 1e-6)
            {
               collisionNormal.normalize();
               SimpleContactWrapper contacts = new SimpleContactWrapper(objectOne, objectTwo);

               double distanceToReport = -pointOnAToPack.distance(pointOnBToPack); //TODO: Do we even need this?
               contacts.addContact(new Point3D(pointOnAToPack), new Point3D(pointOnBToPack), collisionNormal, distanceToReport);
               result.addContact(contacts);
            }

         return true;
      }

      return false;
   }

   public void addShape(CollisionShape collisionShape)
   {
      collisionObjects.add(collisionShape);
   }

   private final Vector3D uVector = new Vector3D();
   private final Vector3D vVector = new Vector3D();
   private final Vector3D w0Vector = new Vector3D();

   public void getClosestPointsOnLineSegments(LineSegment3D segmentOne, LineSegment3D segmentTwo, Point3D closestPointOnOneToPack,
         Point3D closestPointOnTwoToPack)
   {
      Point3DBasics p0 = segmentOne.getFirstEndpoint();
      Point3DBasics p1 = segmentOne.getSecondEndpoint();
      Point3DBasics q0 = segmentTwo.getFirstEndpoint();
      Point3DBasics q1 = segmentTwo.getSecondEndpoint();

      uVector.sub(p1, p0);
      vVector.sub(q1, q0);

      w0Vector.sub(p0, q0);

      double a = uVector.dot(uVector);
      double b = uVector.dot(vVector);
      double c = vVector.dot(vVector);
      double d = uVector.dot(w0Vector);
      double e = vVector.dot(w0Vector);

      double denominator = a * c - b * b;

      double lambdaOne, numeratorOne, denominatorOne = denominator;
      double lambdaTwo, numeratorTwo, denominatorTwo = denominator;

      double smallNumber = 1e-7;

      // compute the line parameters of the two closest points
      if (denominator < smallNumber)
      {
         // the lines are almost parallel
         numeratorOne = 0.0; // force using point P0 on segment S1
         denominatorOne = 1.0; // to prevent possible division by 0.0 later
         numeratorTwo = e;
         denominatorTwo = c;
      }
      else
      {
         // get the closest points on the infinite lines
         numeratorOne = (b * e - c * d);
         numeratorTwo = (a * e - b * d);
         if (numeratorOne < 0.0)
         {
            // sc < 0 => the s=0 edge is visible
            numeratorOne = 0.0;
            numeratorTwo = e;
            denominatorTwo = c;
         }
         else if (numeratorOne > denominatorOne)
         {
            // sc > 1  => the s=1 edge is visible
            numeratorOne = denominatorOne;
            numeratorTwo = e + b;
            denominatorTwo = c;
         }
      }

      if (numeratorTwo < 0.0)
      {
         // tc < 0 => the t=0 edge is visible
         numeratorTwo = 0.0;
         // recompute sc for this edge
         if (-d < 0.0)
            numeratorOne = 0.0;
         else if (-d > a)
            numeratorOne = denominatorOne;
         else
         {
            numeratorOne = -d;
            denominatorOne = a;
         }
      }
      else if (numeratorTwo > denominatorTwo)
      { // tc > 1  => the t=1 edge is visible
         numeratorTwo = denominatorTwo;
         // recompute sc for this edge
         if ((-d + b) < 0.0)
            numeratorOne = 0;
         else if ((-d + b) > a)
            numeratorOne = denominatorOne;
         else
         {
            numeratorOne = (-d + b);
            denominatorOne = a;
         }
      }
      // finally do the division to get sc and tc
      lambdaOne = (Math.abs(numeratorOne) < smallNumber ? 0.0 : numeratorOne / denominatorOne);
      lambdaTwo = (Math.abs(numeratorTwo) < smallNumber ? 0.0 : numeratorTwo / denominatorTwo);

      // get the difference of the two closest points

      closestPointOnOneToPack.set(uVector);
      closestPointOnOneToPack.scaleAdd(lambdaOne, p0);

      closestPointOnTwoToPack.set(vVector);
      closestPointOnTwoToPack.scaleAdd(lambdaTwo, q0);
   }

}
