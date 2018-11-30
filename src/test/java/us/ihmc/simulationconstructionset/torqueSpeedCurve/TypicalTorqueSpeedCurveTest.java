package us.ihmc.simulationconstructionset.torqueSpeedCurve;

import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

import static us.ihmc.robotics.Assert.*;

public class TypicalTorqueSpeedCurveTest
{
   @BeforeEach
   public void setUp() throws Exception
   {
   }

   @AfterEach
   public void tearDown() throws Exception
   {
   }

	@Test// timeout=300000
   public void testOne()
   {
      TypicalTorqueSpeedCurve torqueSpeedCurve = new TypicalTorqueSpeedCurve();
      
      double maxTorque = 100.0;
      double maxSpeed = 12.0;
      double maxSpeedAtMaxTorque = 2.0;
      
      torqueSpeedCurve.setMaxTorque(maxTorque);
      torqueSpeedCurve.setMaxSpeed(maxSpeed);
      torqueSpeedCurve.setMaxSpeedAtMaxTorque(maxSpeedAtMaxTorque);
      
      // Test that you should get 0.0 torque at any speed:
      assertEquals(0.0, torqueSpeedCurve.limitTorque(0.0, 0.0), 1e-7);
      assertEquals(0.0, torqueSpeedCurve.limitTorque(0.0, 10.0 * maxSpeed), 1e-7);
      assertEquals(0.0, torqueSpeedCurve.limitTorque(0.0, -10.0 * maxSpeed), 1e-7);
      
      // Test at zero speed:
      assertEquals(maxTorque, torqueSpeedCurve.limitTorque(2.0 * maxTorque, 0.0), 1e-7);
      assertEquals(-maxTorque, torqueSpeedCurve.limitTorque(-2.0 * maxTorque, 0.0), 1e-7);
      assertEquals(0.5*maxTorque, torqueSpeedCurve.limitTorque(0.5 * maxTorque, 0.0), 1e-7);
      assertEquals(0.5*maxTorque, torqueSpeedCurve.limitTorque(0.5 * maxTorque, 0.0), 1e-7);
      
      // Test at greater than maxSpeed:
      assertEquals(0.0, torqueSpeedCurve.limitTorque(0.5 * maxTorque, maxSpeed), 1e-7);
      assertEquals(0.0, torqueSpeedCurve.limitTorque(0.5 * maxTorque, 1.5 * maxSpeed), 1e-7);
      assertEquals(0.0, torqueSpeedCurve.limitTorque(-0.5 * maxTorque, -maxSpeed), 1e-7);
      assertEquals(0.0, torqueSpeedCurve.limitTorque(-0.5 * maxTorque, -1.5 * maxSpeed), 1e-7);

      // Test at opposite signs:
      assertEquals(0.5 * maxTorque, torqueSpeedCurve.limitTorque(0.5 * maxTorque, -maxSpeed), 1e-7);
      assertEquals(0.5 * maxTorque, torqueSpeedCurve.limitTorque(0.5 * maxTorque, -10.0 * maxSpeed), 1e-7);
      assertEquals(-0.5 * maxTorque, torqueSpeedCurve.limitTorque(-0.5 * maxTorque, maxSpeed), 1e-7);
      assertEquals(-0.5 * maxTorque, torqueSpeedCurve.limitTorque(-0.5 * maxTorque, 10.0 * maxSpeed), 1e-7);
      
      // Test below maxSpeedAtMaxTorque
      assertEquals(maxTorque, torqueSpeedCurve.limitTorque(2.0 * maxTorque, 0.5 * maxSpeedAtMaxTorque), 1e-7);
      assertEquals(maxTorque, torqueSpeedCurve.limitTorque(maxTorque, 0.5 * maxSpeedAtMaxTorque), 1e-7);
      assertEquals(0.5 * maxTorque, torqueSpeedCurve.limitTorque(0.5 * maxTorque, 0.5 * maxSpeedAtMaxTorque), 1e-7);
      assertEquals(-maxTorque, torqueSpeedCurve.limitTorque(-2.0 * maxTorque, -0.5 * maxSpeedAtMaxTorque), 1e-7);
      assertEquals(-maxTorque, torqueSpeedCurve.limitTorque(-maxTorque, -0.5 * maxSpeedAtMaxTorque), 1e-7);
      assertEquals(-0.5 * maxTorque, torqueSpeedCurve.limitTorque(-0.5 * maxTorque, -0.5 * maxSpeedAtMaxTorque), 1e-7);
      
      // Test on Curve:
      assertEquals(maxTorque, torqueSpeedCurve.limitTorque(2.0 * maxTorque, maxSpeedAtMaxTorque), 1e-7);
      assertEquals(0.0, torqueSpeedCurve.limitTorque(2.0 * maxTorque, maxSpeed), 1e-7);
      assertEquals(0.5 * maxTorque, torqueSpeedCurve.limitTorque(2.0 * maxTorque, (maxSpeed + maxSpeedAtMaxTorque)/2.0), 1e-7);

      assertEquals(-maxTorque, torqueSpeedCurve.limitTorque(-2.0 * maxTorque, -maxSpeedAtMaxTorque), 1e-7);
      assertEquals(-0.0, torqueSpeedCurve.limitTorque(-2.0 * maxTorque, -maxSpeed), 1e-7);
      assertEquals(-0.5 * maxTorque, torqueSpeedCurve.limitTorque(-2.0 * maxTorque, -(maxSpeed + maxSpeedAtMaxTorque)/2.0), 1e-7);
      



   }
}
