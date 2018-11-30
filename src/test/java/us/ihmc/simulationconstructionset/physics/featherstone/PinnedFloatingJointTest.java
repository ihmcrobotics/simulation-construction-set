package us.ihmc.simulationconstructionset.physics.featherstone;

import static us.ihmc.robotics.Assert.*;

import java.util.Random;

import org.junit.jupiter.api.Test;

import us.ihmc.commons.RandomNumbers;
import us.ihmc.euclid.Axis;
import us.ihmc.euclid.tools.EuclidCoreRandomTools;
import us.ihmc.euclid.tools.EuclidCoreTestTools;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.simulationconstructionset.FloatingJoint;
import us.ihmc.simulationconstructionset.Joint;
import us.ihmc.simulationconstructionset.Link;
import us.ihmc.simulationconstructionset.OneDegreeOfFreedomJoint;
import us.ihmc.simulationconstructionset.PinJoint;
import us.ihmc.simulationconstructionset.Robot;
import us.ihmc.simulationconstructionset.RobotTest;
import us.ihmc.simulationconstructionset.SliderJoint;
import us.ihmc.simulationconstructionset.UnreasonableAccelerationException;

public class PinnedFloatingJointTest
{
   private static final double EPSILON = 1.0e-12;

   @Test// timeout = 30000
   public void testPinnedFloatingJoint() throws Exception, UnreasonableAccelerationException
   {
      Random random = new Random(39847534);
      double dt = 1.0e-3;

      for (int iteration = 0; iteration < 50; iteration++)
      {
         int numberOneDoFJoints = random.nextInt(15) + 1;
         Robot robotA = new Robot("pinnedRobot");
         FloatingJoint pinnedJointA = new FloatingJoint("pinned", new Vector3D(), robotA);
         pinnedJointA.setPinned(true);
         pinnedJointA.setLink(nextLink(random.nextLong()));
         robotA.addRootJoint(pinnedJointA);

         Robot robotB = new Robot("robotWithoutFloatingJoint");

         Joint parentJointA = pinnedJointA;
         Joint parentJointB = null;

         OneDegreeOfFreedomJoint[] oneDoFJointsA = new OneDegreeOfFreedomJoint[numberOneDoFJoints];
         OneDegreeOfFreedomJoint[] oneDoFJointsB = new OneDegreeOfFreedomJoint[numberOneDoFJoints];

         for (int i = 0; i < numberOneDoFJoints; i++)
         {
            long seed = random.nextLong();

            OneDegreeOfFreedomJoint nextJointA = nextOneDegreeOfFreedomJoint(seed, robotA);
            OneDegreeOfFreedomJoint nextJointB = nextOneDegreeOfFreedomJoint(seed, robotB);
            nextJointA.setLink(nextLink(seed));
            nextJointB.setLink(nextLink(seed));

            parentJointA.addJoint(nextJointA);
            if (parentJointB == null)
               robotB.addRootJoint(nextJointB);
            else
               parentJointB.addJoint(nextJointB);

            oneDoFJointsA[i] = nextJointA;
            oneDoFJointsB[i] = nextJointB;

            parentJointA = nextJointA;
            parentJointB = nextJointB;
         }

         for (int i = 0; i < 100; i++)
         {
            // Make sure that the joint velocity and acceleration are both 
            pinnedJointA.setVelocity(EuclidCoreRandomTools.nextVector3D(random));
            pinnedJointA.setAngularVelocityInBody(EuclidCoreRandomTools.nextVector3D(random));
            pinnedJointA.setAcceleration(EuclidCoreRandomTools.nextVector3D(random));
            pinnedJointA.setAngularAccelerationInBody(EuclidCoreRandomTools.nextVector3D(random));

            for (int jointIndex = 0; jointIndex < numberOneDoFJoints; jointIndex++)
            {
               double tau = random.nextDouble();
               oneDoFJointsA[jointIndex].setTau(tau);
               oneDoFJointsB[jointIndex].setTau(tau);
            }

            robotA.doDynamicsAndIntegrate(dt);
            robotB.doDynamicsAndIntegrate(dt);

            assertEquals(0.0, pinnedJointA.getQx().getValue(), EPSILON);
            assertEquals(0.0, pinnedJointA.getQy().getValue(), EPSILON);
            assertEquals(0.0, pinnedJointA.getQz().getValue(), EPSILON);
            EuclidCoreTestTools.assertQuaternionEquals(new Quaternion(), pinnedJointA.getQuaternion(), EPSILON);

            assertEquals(0.0, pinnedJointA.getQdx().getValue(), EPSILON);
            assertEquals(0.0, pinnedJointA.getQdy().getValue(), EPSILON);
            assertEquals(0.0, pinnedJointA.getQdz().getValue(), EPSILON);
            assertEquals(0.0, pinnedJointA.getAngularVelocityX().getValue(), EPSILON);
            assertEquals(0.0, pinnedJointA.getAngularVelocityY().getValue(), EPSILON);
            assertEquals(0.0, pinnedJointA.getAngularVelocityZ().getValue(), EPSILON);

            assertEquals(0.0, pinnedJointA.getQddx().getValue(), EPSILON);
            assertEquals(0.0, pinnedJointA.getQddy().getValue(), EPSILON);
            assertEquals(0.0, pinnedJointA.getQddz().getValue(), EPSILON);
            assertEquals(0.0, pinnedJointA.getAngularAccelerationX().getValue(), EPSILON);
            assertEquals(0.0, pinnedJointA.getAngularAccelerationY().getValue(), EPSILON);
            assertEquals(0.0, pinnedJointA.getAngularAccelerationZ().getValue(), EPSILON);

            for (int jointIndex = 0; jointIndex < numberOneDoFJoints; jointIndex++)
            {
               assertEquals(oneDoFJointsA[jointIndex].getQ(), oneDoFJointsB[jointIndex].getQ(), EPSILON);
               assertEquals(oneDoFJointsA[jointIndex].getQD(), oneDoFJointsB[jointIndex].getQD(), EPSILON);
               assertEquals(oneDoFJointsA[jointIndex].getQDD(), oneDoFJointsB[jointIndex].getQDD(), EPSILON);
            }
         }
      }
   }

   @Test// timeout = 30000
   public void testUnpinningFloatingJoint() throws Exception, UnreasonableAccelerationException
   {
      Random random = new Random(39847534);
      double dt = 1.0e-3;

      for (int iteration = 0; iteration < 50; iteration++)
      {
         int numberOneDoFJoints = random.nextInt(15) + 1;
         Robot robotA = new Robot("unpinningRobot");
         FloatingJoint floatingJointA = new FloatingJoint("unpinning", new Vector3D(), robotA);
         floatingJointA.setPinned(true);
         long rootLinkSeed = random.nextLong();
         floatingJointA.setLink(nextLink(rootLinkSeed));
         robotA.addRootJoint(floatingJointA);

         Robot robotB = new Robot("robotWithFloatingJoint");
         FloatingJoint floatingJointB = new FloatingJoint("unpinning", new Vector3D(), robotB);
         floatingJointB.setLink(nextLink(rootLinkSeed));
         robotB.addRootJoint(floatingJointB);

         Joint parentJointA = floatingJointA;
         Joint parentJointB = floatingJointB;

         OneDegreeOfFreedomJoint[] oneDoFJointsA = new OneDegreeOfFreedomJoint[numberOneDoFJoints];
         OneDegreeOfFreedomJoint[] oneDoFJointsB = new OneDegreeOfFreedomJoint[numberOneDoFJoints];

         for (int i = 0; i < numberOneDoFJoints; i++)
         {
            long seed = random.nextLong();

            OneDegreeOfFreedomJoint nextJointA = nextOneDegreeOfFreedomJoint(seed, robotA);
            OneDegreeOfFreedomJoint nextJointB = nextOneDegreeOfFreedomJoint(seed, robotB);
            nextJointA.setLink(nextLink(seed));
            nextJointB.setLink(nextLink(seed));

            parentJointA.addJoint(nextJointA);
            parentJointB.addJoint(nextJointB);

            oneDoFJointsA[i] = nextJointA;
            oneDoFJointsB[i] = nextJointB;

            parentJointA = nextJointA;
            parentJointB = nextJointB;
         }

         for (int i = 0; i < 100; i++)
         { // Run dynamics on the robot A
            floatingJointA.setVelocity(EuclidCoreRandomTools.nextVector3D(random));
            floatingJointA.setAngularVelocityInBody(EuclidCoreRandomTools.nextVector3D(random));
            floatingJointA.setAcceleration(EuclidCoreRandomTools.nextVector3D(random));
            floatingJointA.setAngularAccelerationInBody(EuclidCoreRandomTools.nextVector3D(random));

            for (int jointIndex = 0; jointIndex < numberOneDoFJoints; jointIndex++)
            {
               double tau = random.nextDouble();
               oneDoFJointsA[jointIndex].setTau(tau);
            }

            robotA.doDynamicsAndIntegrate(dt);
         }

         // Copy the robot states A => B
         floatingJointB.getQx().set(floatingJointA.getQx().getValue());
         floatingJointB.getQy().set(floatingJointA.getQy().getValue());
         floatingJointB.getQz().set(floatingJointA.getQz().getValue());
         floatingJointB.getQuaternionQs().set(floatingJointA.getQuaternionQs().getValue());
         floatingJointB.getQuaternionQx().set(floatingJointA.getQuaternionQx().getValue());
         floatingJointB.getQuaternionQy().set(floatingJointA.getQuaternionQy().getValue());
         floatingJointB.getQuaternionQz().set(floatingJointA.getQuaternionQz().getValue());

         for (int jointIndex = 0; jointIndex < numberOneDoFJoints; jointIndex++)
         {
            oneDoFJointsB[jointIndex].setQ(oneDoFJointsA[jointIndex].getQ());
            oneDoFJointsB[jointIndex].setQd(oneDoFJointsA[jointIndex].getQD());
            oneDoFJointsB[jointIndex].setQdd(oneDoFJointsA[jointIndex].getQDD());
         }

         floatingJointA.setPinned(false);

         for (int i = 0; i < 100; i++)
         {
            for (int jointIndex = 0; jointIndex < numberOneDoFJoints; jointIndex++)
            {
               double tau = random.nextDouble();
               oneDoFJointsA[jointIndex].setTau(tau);
               oneDoFJointsB[jointIndex].setTau(tau);
            }

            robotA.doDynamicsAndIntegrate(dt);
            robotB.doDynamicsAndIntegrate(dt);

            assertEquals(floatingJointB.getQx().getValue(), floatingJointA.getQx().getValue(), EPSILON);
            assertEquals(floatingJointB.getQy().getValue(), floatingJointA.getQy().getValue(), EPSILON);
            assertEquals(floatingJointB.getQz().getValue(), floatingJointA.getQz().getValue(), EPSILON);
            EuclidCoreTestTools.assertQuaternionEquals(floatingJointB.getQuaternion(), floatingJointA.getQuaternion(), EPSILON);

            assertEquals(floatingJointB.getQdx().getValue(), floatingJointA.getQdx().getValue(), EPSILON);
            assertEquals(floatingJointB.getQdy().getValue(), floatingJointA.getQdy().getValue(), EPSILON);
            assertEquals(floatingJointB.getQdz().getValue(), floatingJointA.getQdz().getValue(), EPSILON);
            assertEquals(floatingJointB.getAngularVelocityX().getValue(), floatingJointA.getAngularVelocityX().getValue(), EPSILON);
            assertEquals(floatingJointB.getAngularVelocityY().getValue(), floatingJointA.getAngularVelocityY().getValue(), EPSILON);
            assertEquals(floatingJointB.getAngularVelocityZ().getValue(), floatingJointA.getAngularVelocityZ().getValue(), EPSILON);

            assertEquals(floatingJointB.getQddx().getValue(), floatingJointA.getQddx().getValue(), EPSILON);
            assertEquals(floatingJointB.getQddy().getValue(), floatingJointA.getQddy().getValue(), EPSILON);
            assertEquals(floatingJointB.getQddz().getValue(), floatingJointA.getQddz().getValue(), EPSILON);
            assertEquals(floatingJointB.getAngularAccelerationX().getValue(), floatingJointA.getAngularAccelerationX().getValue(), EPSILON);
            assertEquals(floatingJointB.getAngularAccelerationY().getValue(), floatingJointA.getAngularAccelerationY().getValue(), EPSILON);
            assertEquals(floatingJointB.getAngularAccelerationZ().getValue(), floatingJointA.getAngularAccelerationZ().getValue(), EPSILON);

            for (int jointIndex = 0; jointIndex < numberOneDoFJoints; jointIndex++)
            {
               assertEquals(oneDoFJointsA[jointIndex].getQ(), oneDoFJointsB[jointIndex].getQ(), EPSILON);
               assertEquals(oneDoFJointsA[jointIndex].getQD(), oneDoFJointsB[jointIndex].getQD(), EPSILON);
               assertEquals(oneDoFJointsA[jointIndex].getQDD(), oneDoFJointsB[jointIndex].getQDD(), EPSILON);
            }
         }
      }
   }

   private static Link nextLink(long seed)
   {
      Random random = new Random(seed);
      Link link = new Link("randomLink" + random.nextInt());
      link.setMass(random.nextDouble());
      link.setComOffset(RandomNumbers.nextDouble(random, 1.0), RandomNumbers.nextDouble(random, 1.0), RandomNumbers.nextDouble(random, 1.0));
      link.setMomentOfInertia(RobotTest.getRotationalInertiaMatrixOfSolidEllipsoid(link.getMass(), random.nextDouble(), random.nextDouble(),
                                                                                   random.nextDouble()));
      return link;
   }

   private static OneDegreeOfFreedomJoint nextOneDegreeOfFreedomJoint(long seed, Robot rob)
   {
      Random random = new Random(seed);
      if (random.nextBoolean())
         return nextPinJoint(seed, rob);
      else
         return nextSliderJoint(seed, rob);
   }

   private static PinJoint nextPinJoint(long seed, Robot rob)
   {
      Random random = new Random(seed);
      String jname = "randomPinJoint" + random.nextInt();
      Vector3DReadOnly offset = EuclidCoreRandomTools.nextVector3D(random);
      Axis jaxis = Axis.values[random.nextInt(Axis.values.length)];
      PinJoint pinJoint = new PinJoint(jname, offset, rob, jaxis);
      pinJoint.setQ(random.nextDouble());
      pinJoint.setQd(random.nextDouble());
      return pinJoint;
   }

   private static SliderJoint nextSliderJoint(long seed, Robot rob)
   {
      Random random = new Random(seed);
      String jname = "randomSliderJoint" + random.nextInt();
      Vector3D offset = EuclidCoreRandomTools.nextVector3D(random);
      Axis jaxis = Axis.values[random.nextInt(Axis.values.length)];
      SliderJoint sliderJoint = new SliderJoint(jname, offset, rob, jaxis);
      sliderJoint.setQ(random.nextDouble());
      sliderJoint.setQd(random.nextDouble());
      return sliderJoint;
   }
}
