package us.ihmc.simulationconstructionset.physics.featherstone;

import static us.ihmc.robotics.Assert.fail;

import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Disabled;
import org.junit.jupiter.api.Test;

import us.ihmc.simulationconstructionset.SimulationConstructionSet;
import us.ihmc.simulationconstructionset.util.RobotController;
import us.ihmc.simulationconstructionset.util.simulationRunner.BlockingSimulationRunner;
import us.ihmc.simulationconstructionset.util.simulationTesting.SimulationTestingParameters;
import us.ihmc.tools.MemoryTools;
import us.ihmc.yoVariables.registry.YoRegistry;

/**
 * Tests simulation against closed-form dynamics
 */
public class FeatherstoneAlgorithmTest
{
   @BeforeEach
   public void setup()
   {
      MemoryTools.printCurrentMemoryUsageAndReturnUsedMemoryInMB(getClass().getSimpleName() + " before test.");
   }

   @AfterEach
   public void tearDown()
   {
      MemoryTools.printCurrentMemoryUsageAndReturnUsedMemoryInMB(getClass().getSimpleName() + " after test.");
   }

   private final SimulationTestingParameters simulationTestingParameters = SimulationTestingParameters.createFromSystemProperties();

   @Test // timeout = 30000
   public void testSinglePendulumAgainstLagrangianCalculation()
   {
      double epsilon = 1e-7;
      SinglePendulumRobot pendulumRobot = new SinglePendulumRobot("pendulum", 1.2, -0.4);
      testAgainstLagrangianCalculation(pendulumRobot, epsilon);
   }

   @Test // timeout = 30000
   public void testDoublePendulumAgainstLagrangianCalculation()
   {
      double epsilon = 1e-6;
      DoublePendulumRobot pendulumRobot = new DoublePendulumRobot("doublePendulum", 1.2, -0.4, -0.2, 0.5);
      testAgainstLagrangianCalculation(pendulumRobot, epsilon);
   }

   @Disabled
   @Test // timeout = 30000
   public void testCartPoleAgainstLagrangianCalculation()
   {
      double epsilon = 1e-2;
      CartPoleRobot cartPoleRobot = new CartPoleRobot("cartPole", 0.3, -1.3, 0.4);
      testAgainstLagrangianCalculation(cartPoleRobot, epsilon);
   }

   @Disabled
   @Test // timeout = 30000
   public void testUniversalJointAgainLagrangianCalculation()
   {
      double epsilon = 1e-4;
      UniversalJointRobot universalJointRobot = new UniversalJointRobot("universalJoint", 0.4, -0.2, -0.4, 0.3);
      testAgainstLagrangianCalculation(universalJointRobot, epsilon);
   }

   private void testAgainstLagrangianCalculation(RobotWithClosedFormDynamics robotWithClosedFormDynamics, double epsilon)
   {
      robotWithClosedFormDynamics.setController(new DynamicsChecker(robotWithClosedFormDynamics, epsilon));

      SimulationConstructionSet scs = new SimulationConstructionSet(robotWithClosedFormDynamics, simulationTestingParameters);
      scs.setDT(1e-5, 20);
      scs.startOnAThread();
      BlockingSimulationRunner blockingSimulationRunner = new BlockingSimulationRunner(scs, 45.0);

      try
      {
         blockingSimulationRunner.simulateAndBlock(15.0);
      }
      catch (Exception e)
      {
         fail();
      }
      scs.closeAndDispose();
   }

   private class DynamicsChecker implements RobotController
   {
      private final YoRegistry registry;
      private final RobotWithClosedFormDynamics robotWithClosedFormDynamics;
      private final double epsilon;
      private int numberOfTicksToWait = 2;

      public DynamicsChecker(RobotWithClosedFormDynamics robotWithClosedFormDynamics, double epsilon)
      {
         registry = new YoRegistry(robotWithClosedFormDynamics.getName() + "Registry");
         this.robotWithClosedFormDynamics = robotWithClosedFormDynamics;
         this.epsilon = epsilon;
      }

      @Override
      public void initialize()
      {

      }

      @Override
      public YoRegistry getYoRegistry()
      {
         return registry;
      }

      @Override
      public String getName()
      {
         return robotWithClosedFormDynamics.getName();
      }

      @Override
      public String getDescription()
      {
         return getName();
      }

      @Override
      public void doControl()
      {
         if (numberOfTicksToWait == 0)
            robotWithClosedFormDynamics.assertStateIsCloseToClosedFormCalculation(epsilon);
         else
            numberOfTicksToWait--;
      }
   }
}
