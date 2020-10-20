package us.ihmc.simulationconstructionset.util;

import static us.ihmc.robotics.Assert.assertEquals;
import static us.ihmc.robotics.Assert.assertFalse;
import static us.ihmc.robotics.Assert.assertTrue;

import java.util.ArrayList;
import java.util.Random;

import org.junit.jupiter.api.Test;

import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.euclid.tools.EuclidCoreRandomTools;
import us.ihmc.euclid.tools.EuclidCoreTestTools;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.simulationconstructionset.GroundContactPoint;
import us.ihmc.simulationconstructionset.GroundContactPointsHolder;
import us.ihmc.simulationconstructionset.Robot;
import us.ihmc.simulationconstructionset.SimulationConstructionSet;
import us.ihmc.simulationconstructionset.SimulationConstructionSetParameters;
import us.ihmc.simulationconstructionset.util.ground.FlatGroundProfile;
import us.ihmc.simulationconstructionset.util.ground.SlopedPlaneGroundProfile;
import us.ihmc.yoVariables.registry.YoRegistry;

public class LinearStickSlipGroundContactModelTest
{
   private static SimulationConstructionSetParameters parameters = SimulationConstructionSetParameters.createFromSystemProperties();

   @Test // timeout=300000
   public void testNonlinearZForce()
   {
      boolean visualize = false;

      SimulationConstructionSet scs = null;
      YoRegistry registry;

      if (visualize)
      {
         scs = new SimulationConstructionSet(new Robot("TestZForce"), parameters);
         registry = scs.getRootRegistry();
      }
      else
      {
         registry = new YoRegistry("TestRegistry");
      }

      GroundContactPoint groundContactPoint = new GroundContactPoint("testPoint", registry);
      GroundContactPointsHolder pointsHolder = createGroundContactPointsHolder(groundContactPoint);

      LinearStickSlipGroundContactModel groundContactModel = new LinearStickSlipGroundContactModel(pointsHolder, registry);
      groundContactModel.disableSlipping();

      if (visualize)
      {
         scs.startOnAThread();
      }

      for (double z = 0.00001; z > -0.02; z = z - 0.00001)
      {
         Point3D position = new Point3D(0.0, 0.0, z);
         Vector3D velocity = new Vector3D(0.0, 0.0, 0.0);

         groundContactPoint.setPosition(position);
         groundContactPoint.setVelocity(velocity);

         groundContactModel.enableSurfaceNormal();
         groundContactModel.doGroundContact();

         Vector3D force = new Vector3D();
         groundContactPoint.getForce(force);

         if (visualize)
         {
            scs.tickAndUpdate();
         }
      }

      if (visualize)
      {
         ThreadTools.sleepForever();
      }
   }

   @Test // timeout=300000
   public void testOnFlatGroundNoSlipCompareWithAndWithoutNormals()
   {
      YoRegistry registry = new YoRegistry("TestRegistry");

      GroundContactPoint groundContactPoint = new GroundContactPoint("testPoint", registry);
      GroundContactPointsHolder pointsHolder = createGroundContactPointsHolder(groundContactPoint);

      LinearStickSlipGroundContactModel groundContactModel = new LinearStickSlipGroundContactModel(pointsHolder, registry);
      groundContactModel.disableSlipping();

      Point3D position = new Point3D(0.0, 0.0, -0.002);
      Vector3D velocity = new Vector3D(0.0, 0.0, -1.0);

      groundContactPoint.setPosition(position);
      groundContactPoint.setVelocity(velocity);

      groundContactModel.enableSurfaceNormal();
      groundContactModel.doGroundContact();

      Vector3D force = new Vector3D();
      groundContactPoint.getForce(force);

      assertEquals(0.0, force.getX(), 1e-7);
      assertEquals(0.0, force.getY(), 1e-7);
      assertTrue(force.getZ() > 0.0);

      Point3D touchdownPosition = new Point3D();
      groundContactPoint.getTouchdownLocation(touchdownPosition);

      EuclidCoreTestTools.assertTuple3DEquals(touchdownPosition, position, 1e-7);

      groundContactModel.disableSurfaceNormal();
      groundContactModel.doGroundContact();

      Vector3D forceWithNormalsDisabled = new Vector3D();
      groundContactPoint.getForce(forceWithNormalsDisabled);

      EuclidCoreTestTools.assertTuple3DEquals(force, forceWithNormalsDisabled, 1e-7);

      int numberOfTests = 1000;

      Random random = new Random(1977L);

      for (int i = 0; i < numberOfTests; i++)
      {
         double maxAbsoluteX = 0.01;
         double maxAbsoluteY = 0.01;
         double maxAbsoluteZ = 0.01;
         double maxSpeed = 0.1;

         position = EuclidCoreRandomTools.nextPoint3D(random, maxAbsoluteX, maxAbsoluteY, maxAbsoluteZ);

         // Keep it under ground for now to make sure touchdown doesn't change.
         if (position.getZ() > -0.002)
            position.setZ(-0.002);

         velocity = EuclidCoreRandomTools.nextVector3DWithFixedLength(random, maxSpeed);
         if (velocity.getZ() > 0.0)
            velocity.setZ(-velocity.getZ());

         groundContactPoint.setPosition(position);
         groundContactPoint.setVelocity(velocity);

         groundContactModel.enableSurfaceNormal();
         groundContactModel.doGroundContact();
         assertTrue(groundContactPoint.isInContact());
         groundContactPoint.getForce(force);

         groundContactModel.disableSurfaceNormal();
         groundContactModel.doGroundContact();
         assertTrue(groundContactPoint.isInContact());
         groundContactPoint.getForce(forceWithNormalsDisabled);

         EuclidCoreTestTools.assertTuple3DEquals(force, forceWithNormalsDisabled, 1e-7);

         Point3D touchdownTest = new Point3D();
         groundContactPoint.getTouchdownLocation(touchdownTest);

         EuclidCoreTestTools.assertTuple3DEquals(touchdownPosition, touchdownTest, 1e-7);
      }

      // Test one above ground:
      position.set(0.2, 0.3, 1e-7);
      velocity.set(0.0, 0.0, 0.0);

      groundContactPoint.setPosition(position);
      groundContactPoint.setVelocity(velocity);

      groundContactModel.enableSurfaceNormal();
      groundContactModel.doGroundContact();
      assertFalse(groundContactPoint.isInContact());
      groundContactPoint.getForce(force);

      EuclidCoreTestTools.assertTuple3DEquals(new Vector3D(0.0, 0.0, 0.0), force, 1e-7);
   }

   @Test // timeout=300000
   public void testOnSlantedGroundCompareWithAndWithoutNormals()
   {
      YoRegistry registryOnFlat = new YoRegistry("TestRegistryOnFlat");
      YoRegistry registryOnSlope = new YoRegistry("TestRegistryOnFlat");

      GroundContactPoint groundContactPointOnFlat = new GroundContactPoint("testPointOnFlat", registryOnFlat);
      GroundContactPointsHolder pointsHolderOnFlat = createGroundContactPointsHolder(groundContactPointOnFlat);

      GroundContactPoint groundContactPointOnSlope = new GroundContactPoint("testPointOnSlope", registryOnSlope);
      GroundContactPointsHolder pointsHolderOnSlope = createGroundContactPointsHolder(groundContactPointOnSlope);

      LinearStickSlipGroundContactModel groundContactModelOnFlat = new LinearStickSlipGroundContactModel(pointsHolderOnFlat, registryOnFlat);
      groundContactModelOnFlat.enableSlipping();

      LinearStickSlipGroundContactModel groundContactModelOnSlope = new LinearStickSlipGroundContactModel(pointsHolderOnSlope, registryOnSlope);
      groundContactModelOnSlope.enableSlipping();

      FlatGroundProfile flatGroundProfile = new FlatGroundProfile();
      groundContactModelOnFlat.setGroundProfile3D(flatGroundProfile);

      RigidBodyTransform transform3D = new RigidBodyTransform();
      transform3D.setRotationRollAndZeroTranslation(0.3);
      transform3D.setRotationPitchAndZeroTranslation(-0.7);
      transform3D.getTranslation().set(new Vector3D(0.1, 0.2, 0.3));

      RigidBodyTransform inverseTransform3D = new RigidBodyTransform(transform3D);
      inverseTransform3D.invert();

      Vector3D surfaceNormal = new Vector3D(0.0, 0.0, 1.0);
      transform3D.transform(surfaceNormal);
      surfaceNormal.normalize();
      Point3D intersectionPoint = new Point3D();
      transform3D.transform(intersectionPoint);

      SlopedPlaneGroundProfile slopedGroundProfile = new SlopedPlaneGroundProfile(surfaceNormal, intersectionPoint, 100.0);
      groundContactModelOnSlope.setGroundProfile3D(slopedGroundProfile);

      Random random = new Random(1833L);

      int numberOfTests = 10000;

      for (int i = 0; i < numberOfTests; i++)
      {
         double maxAbsoluteXYZ = 0.1;
         double maxAbsoluteVelocity = 1.0;
         Point3D queryPointOnFlat = EuclidCoreRandomTools.nextPoint3D(random, maxAbsoluteXYZ, maxAbsoluteXYZ, maxAbsoluteXYZ);
         Vector3D queryVelocityOnFlat = EuclidCoreRandomTools.nextVector3DWithFixedLength(random, maxAbsoluteVelocity);

         groundContactPointOnFlat.setPosition(queryPointOnFlat);
         groundContactPointOnFlat.setVelocity(queryVelocityOnFlat);
         groundContactModelOnFlat.doGroundContact();
         Vector3D forceOnFlat = new Vector3D();
         groundContactPointOnFlat.getForce(forceOnFlat);

         Point3D queryPointOnSlope = new Point3D(queryPointOnFlat);
         Vector3D queryVelocityOnSlope = new Vector3D(queryVelocityOnFlat);

         transform3D.transform(queryPointOnSlope);
         transform3D.transform(queryVelocityOnSlope);

         groundContactPointOnSlope.setPosition(queryPointOnSlope);
         groundContactPointOnSlope.setVelocity(queryVelocityOnSlope);
         groundContactModelOnSlope.doGroundContact();

         Vector3D forceOnSlope = new Vector3D();
         groundContactPointOnSlope.getForce(forceOnSlope);

         inverseTransform3D.transform(forceOnSlope);

         EuclidCoreTestTools.assertTuple3DEquals(forceOnFlat, forceOnSlope, 1e-7);

         assertTrue(groundContactPointOnFlat.isInContact() == groundContactPointOnSlope.isInContact());
         assertTrue(groundContactPointOnFlat.isSlipping() == groundContactPointOnSlope.isSlipping());

      }
   }

   private GroundContactPointsHolder createGroundContactPointsHolder(GroundContactPoint groundContactPoint)
   {
      final ArrayList<GroundContactPoint> groundContactPoints = new ArrayList<>();
      groundContactPoints.add(groundContactPoint);

      GroundContactPointsHolder pointsHolder = new GroundContactPointsHolder()
      {
         @Override
         public ArrayList<GroundContactPoint> getGroundContactPoints(int groundContactGroupIdentifier)
         {
            return groundContactPoints;
         }
      };
      return pointsHolder;
   }

}
