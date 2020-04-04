package us.ihmc.simulationconstructionset;

import org.junit.jupiter.api.Disabled;
import org.junit.jupiter.api.Tag;
import org.junit.jupiter.api.Test;
import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.euclid.Axis3D;
import us.ihmc.simulationconstructionset.examples.FallingBrickRobot;
import us.ihmc.simulationconstructionset.gui.SimulationGUITestFixture;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoEnum;

import static us.ihmc.robotics.Assert.*;

@Tag("gui")
public class SimulationConstructionSetFestTest
{
   private static SimulationConstructionSetParameters parameters = SimulationConstructionSetParameters.createFromSystemProperties();

   private boolean isGradleBuild()
   {
      String property = System.getProperty("bamboo.gradle");
      if (property != null && property.contains("yes"))
      {
         return true;
      }

      return false;
   }
   @Disabled //java.lang.AssertionError: expected:<4> but was:<909>
	@Test// timeout=100000
   public void testSimulationConstructionSetUsingGUITestFixture()
   {
      FallingBrickRobot robot = new FallingBrickRobot();

      SimulationConstructionSet scs = new SimulationConstructionSet(robot, parameters);
      YoVariableRegistry registryOne = new YoVariableRegistry("RegistryOne");
      YoEnum<Axis3D> enumForTests = new YoEnum<Axis3D>("enumForTests", registryOne, Axis3D.class);
      YoVariableRegistry registryTwo = new YoVariableRegistry("RegistryTwo");
      YoBoolean booleanForTests = new YoBoolean("booleanForTests", registryTwo);
      registryOne.addChild(registryTwo);
      scs.addYoVariableRegistry(registryOne);

      scs.setFrameMaximized();
      scs.startOnAThread();
      scs.setSimulateDuration(2.0);
//      scs.hideViewport();


      SimulationGUITestFixture testFixture = new SimulationGUITestFixture(scs);

      testFixture.removeAllGraphs();
      testFixture.removeAllEntryBoxes();

      testFixture.selectNameSpaceTab();
      testFixture.selectNameSpace("root/RegistryOne");
      testFixture.selectVariableInOpenTab("enumForTests");
      ThreadTools.sleep(500);

      testFixture.selectNameSpaceTab();
      testFixture.selectNameSpace("root/RegistryOne/RegistryTwo");
      testFixture.selectVariableInOpenTab("booleanForTests");
      ThreadTools.sleep(500);

      testFixture.clickOnAddNumericEntryBox();
      testFixture.clickOnUnusedEntryBox();

      assertTrue(booleanForTests.getBooleanValue() == false);
      testFixture.findEntryBoxAndEnterValue("booleanForTests", 1.0);
      assertTrue(booleanForTests.getBooleanValue() == true);


      testFixture.selectSearchTab();
      testFixture.enterSearchText("q_");

      testFixture.selectVariableInSearchTab("q_y");
      testFixture.clickNewGraphButton();
      testFixture.middleClickInEmptyGraph();

      testFixture.selectVariableInSearchTab("q_z");
      testFixture.clickNewGraphButton();
      testFixture.middleClickInEmptyGraph();

      testFixture.removeAllGraphs();


      // Setup a few entry boxes:
      enumForTests.set(Axis3D.X);

      testFixture.selectSearchTab();
      testFixture.deleteSearchText();
      testFixture.enterSearchText("enumForTests");
      testFixture.selectVariableInSearchTab("enumForTests");

      testFixture.clickOnAddNumericEntryBox();
      testFixture.clickOnUnusedEntryBox();

      assertTrue(enumForTests.getEnumValue() == Axis3D.X);
      testFixture.findEnumEntryBoxAndSelectValue("enumForTests", "Z");
      assertTrue(enumForTests.getEnumValue() == Axis3D.Z);

      // Search for variables, change their values, and plot them:
//    testFixture.selectNameSpaceTab();
//    ThreadTools.sleep(1000);

      testFixture.selectSearchTab();

      testFixture.deleteSearchText();
      testFixture.enterSearchText("q_");

      testFixture.selectVariableInSearchTab("q_x");
      testFixture.clickRemoveEmptyGraphButton();
      testFixture.clickNewGraphButton();
      testFixture.middleClickInEmptyGraph();

      testFixture.clickNewGraphButton();
      testFixture.clickNewGraphButton();
      testFixture.selectVariableInSearchTab("q_y");
      testFixture.middleClickInNthGraph(2);
      ThreadTools.sleep(500);
      testFixture.selectVariableInSearchTab("q_z");
      testFixture.middleClickInNthGraph(2);

      testFixture.selectVariableAndSetValueInSearchTab("q_z", 1.31);
      YoDouble q_z = (YoDouble) scs.getVariable("q_z");
      assertEquals(1.31, q_z.getDoubleValue(), 1e-9);

      // Simulate and replay
      ThreadTools.sleep(500);
      testFixture.clickSimulateButton();
      ThreadTools.sleep(500);
      testFixture.clickStopButton();
      ThreadTools.sleep(500);
      testFixture.clickPlayButton();
      ThreadTools.sleep(500);
      testFixture.clickStopButton();
      ThreadTools.sleep(500);

      // Remove variables from graphs:
      testFixture.removeVariableFromNthGraph("q_y", 2);
      testFixture.clickRemoveEmptyGraphButton();


      // Go to In/out points, step through data. Add KeyPoints, Verify at the expected indices
      testFixture.clickGotoInPointButton();

      ThreadTools.sleep(100);

      int index = scs.getIndex();
      int inPoint = scs.getInPoint();
      assertEquals(index, inPoint);

      // Do some stepping forwards and putting in key points:
      int stepsForward = 4;
      for (int i = 0; i < stepsForward; i++)
      {
         testFixture.clickStepForwardButton();
      }

      ThreadTools.sleep(100);
      index = scs.getIndex();
      assertEquals(stepsForward, index);
      testFixture.clickAddKeyPointButton();

      for (int i = 0; i < stepsForward; i++)
      {
         testFixture.clickStepForwardButton();
      }

      ThreadTools.sleep(100);
      index = scs.getIndex();
      assertEquals(2 * stepsForward, index);
      testFixture.clickAddKeyPointButton();

      for (int i = 0; i < stepsForward; i++)
      {
         testFixture.clickStepForwardButton();
      }

      ThreadTools.sleep(100);
      index = scs.getIndex();
      assertEquals(3 * stepsForward, index);
      testFixture.clickAddKeyPointButton();

      // Zoom in and out
      testFixture.clickZoomInButton();
      testFixture.clickZoomInButton();
      testFixture.clickZoomInButton();
      testFixture.clickZoomInButton();
      testFixture.clickZoomOutButton();

      testFixture.clickGotoInPointButton();
      testFixture.clickToggleKeyModeButton();

      testFixture.clickStepForwardButton();
      ThreadTools.sleep(100);
      index = scs.getIndex();
      assertEquals(stepsForward, index);

      testFixture.clickStepForwardButton();
      ThreadTools.sleep(100);
      index = scs.getIndex();
      assertEquals(2 * stepsForward, index);

      // Toggle a keypoint off:
      testFixture.clickAddKeyPointButton();

      testFixture.clickStepBackwardButton();
      ThreadTools.sleep(100);
      index = scs.getIndex();
      assertEquals(stepsForward, index);

      testFixture.clickSetInPointButton();
      testFixture.clickStepForwardButton();
      testFixture.clickSetOutPointButton();

      testFixture.clickGotoInPointButton();
      ThreadTools.sleep(100);
      index = scs.getIndex();
      assertEquals(stepsForward, index);

      testFixture.clickGotoOutPointButton();
      ThreadTools.sleep(100);
      index = scs.getIndex();
      assertEquals(3 * stepsForward, index);
      testFixture.clickGotoInPointButton();

      testFixture.clickToggleKeyModeButton();
      testFixture.clickStepForwardButton();
      ThreadTools.sleep(100);
      index = scs.getIndex();
      assertEquals(stepsForward + 1, index);

      testFixture.closeAndDispose();
      scs.closeAndDispose();
      scs = null;
      testFixture = null;
   }
   @Disabled  //org.fest.swing.exception.ComponentLookupException: Unable to find component using matcher
            // us.ihmc.simulationconstructionset.gui.SimulationGUITestFixture$JSpinnerNameEndsWithMatcher@22212533.
	@Test// timeout=45000
   public void testSimulationConstructionSetNewGraphWindowUsingGUITestFixture()
   {
      FallingBrickRobot robot = new FallingBrickRobot();
      robot.initialize();

      SimulationConstructionSet scs = new SimulationConstructionSet(robot, parameters);
      scs.setDT(0.0001, 100);
      scs.startOnAThread();

      SimulationGUITestFixture testFixture = new SimulationGUITestFixture(scs);

      testFixture.clickNewGraphButton();

      testFixture.focusMainSCSWindow();
      testFixture.selectSearchTab();
      testFixture.deleteSearchText();
      testFixture.enterSearchText("t");
      testFixture.selectVariableInSearchTab("t");

      testFixture.middleClickInEmptyGraph();

      testFixture.clickSimulateButton();

      ThreadTools.sleep(200);

      testFixture.clickStopButton();

      testFixture.selectNewGraphWindowMenu();
      testFixture.selectNewGraphWindowMenu();

      testFixture.focusNthGraphArrayWindow(0);
      testFixture.clickNewGraphButton();

      testFixture.focusMainSCSWindow();
      testFixture.selectSearchTab();
      testFixture.deleteSearchText();
      testFixture.enterSearchText("q_");
      testFixture.selectVariableInSearchTab("q_z");

      testFixture.focusNthGraphArrayWindow(0);
      testFixture.middleClickInEmptyGraph();

      testFixture.focusMainSCSWindow();
      testFixture.selectSearchTab();
      testFixture.deleteSearchText();
      testFixture.enterSearchText("q_");
      testFixture.selectVariableInSearchTab("q_y");

      testFixture.focusNthGraphArrayWindow(1);
      testFixture.clickNewGraphButton();
      testFixture.middleClickInEmptyGraph();

      testFixture.closeAndDispose();
      scs.closeAndDispose();
      scs = null;
      testFixture = null;
   }
}
