package us.ihmc.simulationconstructionset;

import org.junit.Assume;
import org.junit.jupiter.api.Disabled;
import org.junit.jupiter.api.Test;
import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.simulationconstructionset.examples.FallingBrickRobot;
import us.ihmc.simulationconstructionset.gui.SimulationGUITestFixture;

import java.awt.*;

public class SimulationConstructionSetTest
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

   @Disabled //org.junit.runners.model.TestTimedOutException: test timed out after 300000 milliseconds
	@Test// timeout=300000
   public void testSimulationConstructionSetNewViewportWindowUsingGUITestFixture() throws AWTException
   {
      Assume.assumeTrue(!isGradleBuild());
      FallingBrickRobot robot = new FallingBrickRobot();

      SimulationConstructionSet scs = new SimulationConstructionSet(robot, parameters);
      scs.setDT(0.0001, 100);
      scs.setFrameMaximized();
      scs.startOnAThread();
      scs.setSimulateDuration(2.0);

      ThreadTools.sleep(2000);
      SimulationGUITestFixture testFixture = new SimulationGUITestFixture(scs);
      
      testFixture.closeAllViewportWindows();
      testFixture.selectNewViewportWindowMenu();
      
      testFixture.focusNthViewportWindow(0);

      ThreadTools.sleepForever();
      
      testFixture.closeAndDispose();
      scs.closeAndDispose();
      scs = null;
      testFixture = null;

   }

   @Disabled //org.junit.runners.model.TestTimedOutException: test timed out after 300000 milliseconds
	@Test// timeout=300000
   public void testSimulationConstructionSetVideoGenerationUsingGUITestFixture() throws AWTException
   {
      Assume.assumeTrue(!isGradleBuild());
      FallingBrickRobot robot = new FallingBrickRobot();

      SimulationConstructionSet scs = new SimulationConstructionSet(robot, parameters);
      scs.setDT(0.0001, 100);
      scs.setFrameMaximized();
      scs.startOnAThread();
      scs.setSimulateDuration(2.0);

      SimulationGUITestFixture testFixture = new SimulationGUITestFixture(scs);
      testFixture.clickSimulateButton();
      ThreadTools.sleep(1000);
      
      testFixture.clickMediaCaptureButton();

      testFixture.focusDialog("Export Video");
      testFixture.clickPlayButton();
      
      ThreadTools.sleepForever();

      testFixture.closeAndDispose();
      scs.closeAndDispose();
      scs = null;
      testFixture = null;
   }
}
