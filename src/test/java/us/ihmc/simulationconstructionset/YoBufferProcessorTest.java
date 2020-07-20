package us.ihmc.simulationconstructionset;

import static us.ihmc.robotics.Assert.assertEquals;

import org.junit.jupiter.api.Test;

import us.ihmc.simulationconstructionset.util.ControllerFailureException;
import us.ihmc.simulationconstructionset.util.simulationRunner.BlockingSimulationRunner;
import us.ihmc.simulationconstructionset.util.simulationRunner.BlockingSimulationRunner.SimulationExceededMaximumTimeException;
import us.ihmc.yoVariables.buffer.interfaces.YoBufferProcessor;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoDouble;

public class YoBufferProcessorTest
{
   private static final boolean SHOW_GUI = false;

   @Test // timeout=300000
   public void testSimpleDataProcessingFunction() throws SimulationExceededMaximumTimeException, ControllerFailureException
   {
      final Robot robot = new Robot("DataProcessingFunctionTestRobot");

      YoRegistry registry = new YoRegistry(getClass().getSimpleName());

      final YoDouble variableOne = new YoDouble("variableOne", registry);
      final YoDouble variableTwo = new YoDouble("variableTwo", registry);
      final YoDouble variableThree = new YoDouble("variableThree", registry);

      robot.addYoRegistry(registry);

      SimulationConstructionSetParameters parameters = SimulationConstructionSetParameters.createFromSystemProperties();
      parameters.setCreateGUI(SHOW_GUI);
      SimulationConstructionSet scs = new SimulationConstructionSet(robot, parameters);
      scs.setDT(0.001, 1);

      scs.startOnAThread();

      BlockingSimulationRunner runner = new BlockingSimulationRunner(scs, 100.0);
      runner.simulateAndBlock(2.0);

      YoBufferProcessor dataProcessingFunction = new YoBufferProcessor()
      {
         @Override
         public void process(int startIndex, int endIndex, int currentIndex)
         {
            double time = robot.getTime();

            variableOne.set(time);
            variableTwo.set(1.1);
            variableThree.set(9.23);
         }
      };

      scs.tickAndReadFromBuffer(50);
      assertEquals(variableOne.getDoubleValue(), 0.0, 1e-7);
      assertEquals(variableTwo.getDoubleValue(), 0.0, 1e-7);
      assertEquals(variableThree.getDoubleValue(), 0.0, 1e-7);

      scs.applyDataProcessingFunction(dataProcessingFunction);

      scs.gotoInPointNow();
      scs.tickAndReadFromBuffer(500);

      assertEquals(variableOne.getDoubleValue(), 0.5, 1e-7);
      assertEquals(variableOne.getDoubleValue(), robot.getTime(), 1e-7);
      assertEquals(variableTwo.getDoubleValue(), 1.1, 1e-7);
      assertEquals(variableThree.getDoubleValue(), 9.23, 1e-7);

      scs.closeAndDispose();
   }

}
