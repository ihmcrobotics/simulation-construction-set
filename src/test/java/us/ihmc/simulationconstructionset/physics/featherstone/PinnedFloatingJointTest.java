package us.ihmc.simulationconstructionset.physics.featherstone;

import static org.junit.Assert.*;

import java.util.Random;

import org.junit.Test;

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

   @Test(timeout = 30000)
   public void testPinnedFloatingJoint() throws Exception, UnreasonableAccelerationException
   {
      Random random = new Random(39847534);
         double dt = 1.0e-3;

      for (int iteration = 0; iteration < 50; iteration++)
      {
         int numberOneDoFJoints = random.nextInt(15) + 1;
         Robot robotA = new Robot("pinnedRobot");
         FloatingJoint pinnedJoint = new FloatingJoint("pinned", new Vector3D(), robotA);
         pinnedJoint.setPinned(true);
         pinnedJoint.setLink(nextLink(random.nextLong()));
         robotA.addRootJoint(pinnedJoint);

         Robot robotB = new Robot("robotWithFloatingJoint");

         Joint parentJointA = pinnedJoint;
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
            pinnedJoint.setVelocity(EuclidCoreRandomTools.nextVector3D(random));
            pinnedJoint.setAngularVelocityInBody(EuclidCoreRandomTools.nextVector3D(random));
            pinnedJoint.setAcceleration(EuclidCoreRandomTools.nextVector3D(random));
            pinnedJoint.setAngularAccelerationInBody(EuclidCoreRandomTools.nextVector3D(random));

            for (int jointIndex = 0; jointIndex < numberOneDoFJoints; jointIndex++)
            {
               double tau = random.nextDouble();
               oneDoFJointsA[jointIndex].setTau(tau);
               oneDoFJointsB[jointIndex].setTau(tau);
            }

            robotA.doDynamicsAndIntegrate(dt);
            robotB.doDynamicsAndIntegrate(dt);

            assertEquals(0.0, pinnedJoint.getQx().getValue(), EPSILON);
            assertEquals(0.0, pinnedJoint.getQy().getValue(), EPSILON);
            assertEquals(0.0, pinnedJoint.getQz().getValue(), EPSILON);
            EuclidCoreTestTools.assertQuaternionEquals(new Quaternion(), pinnedJoint.getQuaternion(), EPSILON);

            assertEquals(0.0, pinnedJoint.getQdx().getValue(), EPSILON);
            assertEquals(0.0, pinnedJoint.getQdy().getValue(), EPSILON);
            assertEquals(0.0, pinnedJoint.getQdz().getValue(), EPSILON);
            assertEquals(0.0, pinnedJoint.getAngularVelocityX().getValue(), EPSILON);
            assertEquals(0.0, pinnedJoint.getAngularVelocityY().getValue(), EPSILON);
            assertEquals(0.0, pinnedJoint.getAngularVelocityZ().getValue(), EPSILON);

            assertEquals(0.0, pinnedJoint.getQddx().getValue(), EPSILON);
            assertEquals(0.0, pinnedJoint.getQddy().getValue(), EPSILON);
            assertEquals(0.0, pinnedJoint.getQddz().getValue(), EPSILON);
            assertEquals(0.0, pinnedJoint.getAngularAccelerationX().getValue(), EPSILON);
            assertEquals(0.0, pinnedJoint.getAngularAccelerationY().getValue(), EPSILON);
            assertEquals(0.0, pinnedJoint.getAngularAccelerationZ().getValue(), EPSILON);

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
