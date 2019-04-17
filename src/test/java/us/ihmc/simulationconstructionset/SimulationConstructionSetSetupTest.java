package us.ihmc.simulationconstructionset;

import org.junit.jupiter.api.Tag;
import org.junit.jupiter.api.Test;
import us.ihmc.simulationconstructionset.gui.SplashPanel;

import javax.swing.*;

@Tag("gui")
public class SimulationConstructionSetSetupTest
{
   private static SimulationConstructionSetParameters parameters = SimulationConstructionSetParameters.createFromSystemProperties();
   private static final int pauseTimeForGUIs = 5000;

	@Test
   public void testSplashScreen() throws Exception
    {
      SwingUtilities.invokeAndWait(new Runnable()
      {
         @Override
         public void run()
         {
            SplashPanel splashPanel = new SplashPanel();
            JWindow window = splashPanel.showSplashScreen();

            sleep(pauseTimeForGUIs);
            window.dispose();
         }
      });
   }

	@Test
   public void testSimulationConstructionSetWithoutARobot()
   {
      SimulationConstructionSet scs = new SimulationConstructionSet(parameters);
      Thread thread = new Thread(scs);
      thread.start();

      sleep(pauseTimeForGUIs);
      scs.closeAndDispose();
   }

	@Test
   public void testSimulationConstructionSetWithARobot()
   {
      Robot robot = new Robot("NullRobot");
      SimulationConstructionSet scs = new SimulationConstructionSet(robot, parameters);
      Thread thread = new Thread(scs);
      thread.start();

      sleep(pauseTimeForGUIs);
      scs.closeAndDispose();
   }

   private void sleep(long sleepMillis)
   {
      try
      {
         Thread.sleep(sleepMillis);
      }
      catch (InterruptedException e)
      {
      }
   }
}
