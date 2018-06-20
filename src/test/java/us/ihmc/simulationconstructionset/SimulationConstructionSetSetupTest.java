package us.ihmc.simulationconstructionset;

import javax.swing.JWindow;
import javax.swing.SwingUtilities;

import org.junit.Test;

import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationPlan;
import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationTest;
import us.ihmc.continuousIntegration.IntegrationCategory;
import us.ihmc.simulationconstructionset.Robot;
import us.ihmc.simulationconstructionset.SimulationConstructionSet;
import us.ihmc.simulationconstructionset.gui.SplashPanel;

@ContinuousIntegrationPlan(categories = {IntegrationCategory.UI})
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
