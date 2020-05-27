package us.ihmc.simulationconstructionset;

import static us.ihmc.robotics.Assert.assertEquals;

import org.junit.jupiter.api.Test;

import us.ihmc.simulationconstructionset.util.ControllerFailureException;
import us.ihmc.simulationconstructionset.util.simulationRunner.BlockingSimulationRunner;
import us.ihmc.simulationconstructionset.util.simulationRunner.BlockingSimulationRunner.SimulationExceededMaximumTimeException;
import us.ihmc.yoVariables.dataBuffer.DataProcessingFunction;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoDouble;

public class DataProcessingFunctionTest
{
   private static final boolean SHOW_GUI = false;

   @Test // timeout=300000
   public void testSimpleDataProcessingFunction() throws SimulationExceededMaximumTimeException, ControllerFailureException
   {
      final Robot robot = new Robot("DataProcessingFunctionTestRobot");

      YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());

      final YoDouble variableOne = new YoDouble("variableOne", registry);
      final YoDouble variableTwo = new YoDouble("variableTwo", registry);
      final YoDouble variableThree = new YoDouble("variableThree", registry);

      robot.addYoVariableRegistry(registry);

      SimulationConstructionSetParameters parameters = SimulationConstructionSetParameters.createFromSystemProperties();
      parameters.setCreateGUI(SHOW_GUI);
      SimulationConstructionSet scs = new SimulationConstructionSet(robot, parameters);
      scs.setDT(0.001, 1);

      scs.startOnAThread();

      BlockingSimulationRunner runner = new BlockingSimulationRunner(scs, 100.0);
      runner.simulateAndBlock(2.0);

      DataProcessingFunction dataProcessingFunction = new DataProcessingFunction()
      {
         @Override
         public void initializeProcessing()
         {
         }

         @Override
         public void processData()
         {
            double time = robot.getTime();

            variableOne.set(time);
            variableTwo.set(1.1);
            variableThree.set(9.23);
         }
      };

      scs.tick(50);
      assertEquals(variableOne.getDoubleValue(), 0.0, 1e-7);
      assertEquals(variableTwo.getDoubleValue(), 0.0, 1e-7);
      assertEquals(variableThree.getDoubleValue(), 0.0, 1e-7);

      scs.applyDataProcessingFunction(dataProcessingFunction);

      scs.gotoInPointNow();
      scs.tick(500);

      assertEquals(variableOne.getDoubleValue(), 0.5, 1e-7);
      assertEquals(variableOne.getDoubleValue(), robot.getTime(), 1e-7);
      assertEquals(variableTwo.getDoubleValue(), 1.1, 1e-7);
      assertEquals(variableThree.getDoubleValue(), 9.23, 1e-7);
   }

}
