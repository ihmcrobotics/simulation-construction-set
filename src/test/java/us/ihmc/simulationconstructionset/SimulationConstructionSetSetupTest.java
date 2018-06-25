package us.ihmc.simulationconstructionset;

import org.junit.Test;
import us.ihmc.simulationconstructionset.gui.SplashPanel;

import javax.swing.*;

public class SimulationConstructionSetSetupTest
{
   private static SimulationConstructionSetParameters parameters = SimulationConstructionSetParameters.createFromSystemProperties();
   private static final int pauseTimeForGUIs = 5000;

	@Test(timeout = 30000)
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

	@Test(timeout = 30000)
   public void testSimulationConstructionSetWithoutARobot()
   {
      SimulationConstructionSet scs = new SimulationConstructionSet(parameters);
      Thread thread = new Thread(scs);
      thread.start();

      sleep(pauseTimeForGUIs);
      scs.closeAndDispose();
   }

	@Test(timeout = 30000)
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
