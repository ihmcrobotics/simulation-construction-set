package us.ihmc.simulationconstructionset;

import static us.ihmc.robotics.Assert.assertEquals;
import static us.ihmc.robotics.Assert.assertTrue;

import java.util.Random;

import org.junit.jupiter.api.Test;

import us.ihmc.commons.RandomNumbers;
import us.ihmc.euclid.Axis3D;
import us.ihmc.euclid.tools.EuclidCoreTestTools;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple3D.interfaces.Tuple3DBasics;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoDouble;

public class JointWrenchSensorTest
{

   @Test // timeout=300000
   public void testStaticallyHangingMasses() throws UnreasonableAccelerationException
   {
      double massOne = 7.21;
      double massTwo = 8.64;
      Vector3D sensorOffsetFromJointOne = new Vector3D(0.0, 0.0, -0.1);
      Vector3D sensorOffsetFromJointTwo = new Vector3D(0.0, 0.0, -0.1);

      Robot robot = new Robot("JointWrenchSensorTest");

      PinJoint pinJointOne = createPinJointWithHangingMass("pinJointOne", massOne, Axis3D.Y, robot);
      JointWrenchSensor jointWrenchSensorOne = new JointWrenchSensor("jointWrenchSensorOne", sensorOffsetFromJointOne, robot);
      pinJointOne.addJointWrenchSensor(jointWrenchSensorOne);
      assertTrue(jointWrenchSensorOne == pinJointOne.getJointWrenchSensor());
      robot.addRootJoint(pinJointOne);

      PinJoint pinJointTwo = createPinJointWithHangingMass("pinJointTwo", massTwo, Axis3D.X, robot);
      JointWrenchSensor jointWrenchSensorTwo = new JointWrenchSensor("jointWrenchSensorTwo", sensorOffsetFromJointTwo, robot);
      pinJointTwo.addJointWrenchSensor(jointWrenchSensorTwo);
      assertTrue(jointWrenchSensorTwo == pinJointTwo.getJointWrenchSensor());
      pinJointOne.addJoint(pinJointTwo);

      robot.doDynamicsButDoNotIntegrate();

      Vector3D expectedJointForce = new Vector3D(0.0, 0.0, (massOne + massTwo) * robot.getGravityZ());
      Vector3D expectedJointTorque = new Vector3D();
      assertJointWrenchEquals(jointWrenchSensorOne, expectedJointForce, expectedJointTorque);

      expectedJointForce = new Vector3D(0.0, 0.0, (massTwo) * robot.getGravityZ());
      expectedJointTorque = new Vector3D();
      assertJointWrenchEquals(jointWrenchSensorTwo, expectedJointForce, expectedJointTorque);

      assertJointWrenchSensorConsistency(robot, jointWrenchSensorOne);
      assertJointWrenchSensorConsistency(robot, jointWrenchSensorTwo);

      pinJointOne.setQ(Math.PI);
      robot.doDynamicsButDoNotIntegrate();

      expectedJointForce = new Vector3D(0.0, 0.0, -(massOne + massTwo) * robot.getGravityZ());
      expectedJointTorque = new Vector3D();
      assertJointWrenchEquals(jointWrenchSensorOne, expectedJointForce, expectedJointTorque);

      expectedJointForce = new Vector3D(0.0, 0.0, -(massTwo) * robot.getGravityZ());
      expectedJointTorque = new Vector3D();
      assertJointWrenchEquals(jointWrenchSensorTwo, expectedJointForce, expectedJointTorque);
   }

   @Test // timeout=300000
   public void testJointTorquesMatchWhenSensorAtJoint() throws UnreasonableAccelerationException
   {
      double massOne = 7.21;
      double massTwo = 8.64;
      Vector3D sensorOffsetFromJointOne = new Vector3D(0.0, 0.017, 0.0);
      Vector3D sensorOffsetFromJointTwo = new Vector3D(0.015, 0.0, 0.0);

      Robot robot = new Robot("JointWrenchSensorTest");

      PinJoint pinJointOne = createPinJointWithHangingMass("pinJointOne", massOne, Axis3D.Y, robot);
      JointWrenchSensor jointWrenchSensorOne = new JointWrenchSensor("jointWrenchSensorOne", sensorOffsetFromJointOne, robot);
      pinJointOne.addJointWrenchSensor(jointWrenchSensorOne);
      assertTrue(jointWrenchSensorOne == pinJointOne.getJointWrenchSensor());
      robot.addRootJoint(pinJointOne);

      PinJoint pinJointTwo = createPinJointWithHangingMass("pinJointTwo", massTwo, Axis3D.X, robot);
      JointWrenchSensor jointWrenchSensorTwo = new JointWrenchSensor("jointWrenchSensorTwo", sensorOffsetFromJointTwo, robot);
      pinJointTwo.addJointWrenchSensor(jointWrenchSensorTwo);
      assertTrue(jointWrenchSensorTwo == pinJointTwo.getJointWrenchSensor());
      pinJointOne.addJoint(pinJointTwo);

      Tuple3DBasics jointTorque = new Vector3D();

      Random random = new Random(1797L);
      pinJointOne.setQ(RandomNumbers.nextDouble(random, -Math.PI, Math.PI));
      pinJointTwo.setQ(RandomNumbers.nextDouble(random, -Math.PI, Math.PI));

      pinJointOne.setQd(RandomNumbers.nextDouble(random, -1.0, 1.0));
      pinJointTwo.setQd(RandomNumbers.nextDouble(random, -1.0, 1.0));

      for (int i = 0; i < 100; i++)
      {
         pinJointOne.setTau(RandomNumbers.nextDouble(random, -1.0, 1.0));
         pinJointTwo.setTau(RandomNumbers.nextDouble(random, -1.0, 1.0));

         robot.doDynamicsAndIntegrate(0.0001);

         jointWrenchSensorOne.getJointTorque(jointTorque);
         assertEquals(pinJointOne.getTauYoVariable().getDoubleValue(), -jointTorque.getY(), 1e-7);

         jointWrenchSensorTwo.getJointTorque(jointTorque);
         assertEquals(pinJointTwo.getTauYoVariable().getDoubleValue(), -jointTorque.getX(), 1e-7);
      }
   }

   @Test // timeout=300000
   public void testOffsetAtCenterOfMassWithCantileveredBeam() throws UnreasonableAccelerationException
   {
      double massOne = 7.21;

      Robot robot = new Robot("JointWrenchSensorTest");

      PinJoint pinJointOne = createPinJointWithHangingMass("pinJointOne", massOne, Axis3D.Y, robot);

      Vector3D comOffsetFromJointOne = new Vector3D();
      pinJointOne.getLink().getComOffset(comOffsetFromJointOne);

      JointWrenchSensor jointWrenchSensorOne = new JointWrenchSensor("jointWrenchSensorOne", comOffsetFromJointOne, robot);
      pinJointOne.addJointWrenchSensor(jointWrenchSensorOne);
      assertTrue(jointWrenchSensorOne == pinJointOne.getJointWrenchSensor());
      robot.addRootJoint(pinJointOne);

      pinJointOne.setQ(Math.PI / 2.0);
      pinJointOne.setTau(massOne * robot.getGravityZ() * comOffsetFromJointOne.getZ());

      robot.doDynamicsAndIntegrate(0.0001);

      double jointAcceleration = pinJointOne.getQDDYoVariable().getDoubleValue();
      assertEquals(0.0, jointAcceleration, 1e-7);

      Vector3D expectedJointForce = new Vector3D(-massOne * robot.getGravityZ(), 0.0, 0.0);
      Vector3D expectedJointTorque = new Vector3D();
      assertJointWrenchEquals(jointWrenchSensorOne, expectedJointForce, expectedJointTorque);
   }

   private void assertJointWrenchSensorConsistency(Robot robot, JointWrenchSensor jointWrenchSensor)
   {
      Tuple3DBasics jointForce = new Vector3D();
      Tuple3DBasics jointTorque = new Vector3D();

      jointWrenchSensor.getJointForce(jointForce);
      jointWrenchSensor.getJointTorque(jointTorque);

      String name = jointWrenchSensor.getName();

      YoRegistry robotsYoRegistry = robot.getRobotsYoRegistry();
      YoDouble forceX = (YoDouble) robotsYoRegistry.findVariable(name + "_fX");
      YoDouble forceY = (YoDouble) robotsYoRegistry.findVariable(name + "_fY");
      YoDouble forceZ = (YoDouble) robotsYoRegistry.findVariable(name + "_fZ");

      YoDouble torqueX = (YoDouble) robotsYoRegistry.findVariable(name + "_tX");
      YoDouble torqueY = (YoDouble) robotsYoRegistry.findVariable(name + "_tY");
      YoDouble torqueZ = (YoDouble) robotsYoRegistry.findVariable(name + "_tZ");

      assertEquals(jointForce.getX(), forceX.getDoubleValue(), 1e-7);
      assertEquals(jointForce.getY(), forceY.getDoubleValue(), 1e-7);
      assertEquals(jointForce.getZ(), forceZ.getDoubleValue(), 1e-7);

      assertEquals(jointTorque.getX(), torqueX.getDoubleValue(), 1e-7);
      assertEquals(jointTorque.getY(), torqueY.getDoubleValue(), 1e-7);
      assertEquals(jointTorque.getZ(), torqueZ.getDoubleValue(), 1e-7);
   }

   private void assertJointWrenchEquals(JointWrenchSensor jointWrenchSensor, Vector3D expectedJointForce, Vector3D expectedJointTorque)
   {
      Tuple3DBasics jointForce = new Vector3D();
      Tuple3DBasics jointTorque = new Vector3D();

      jointWrenchSensor.getJointForce(jointForce);
      jointWrenchSensor.getJointTorque(jointTorque);

      EuclidCoreTestTools.assertEquals(expectedJointForce, jointForce, 1e-7);
      EuclidCoreTestTools.assertEquals(expectedJointTorque, jointTorque, 1e-7);
   }

   private PinJoint createPinJointWithHangingMass(String name, double mass, Axis3D axis, Robot robot)
   {
      PinJoint pinJoint = new PinJoint(name, new Vector3D(), robot, axis);

      Vector3D comOffset = new Vector3D(0.0, 0.0, -1.0);
      Link linkOne = new Link("link");
      linkOne.setMass(mass);
      linkOne.setComOffset(comOffset);
      linkOne.setMassAndRadiiOfGyration(mass, 0.1, 0.1, 0.1);
      pinJoint.setLink(linkOne);

      return pinJoint;
   }

}
