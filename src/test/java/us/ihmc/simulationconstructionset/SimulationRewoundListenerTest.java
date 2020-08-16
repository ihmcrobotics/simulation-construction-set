package us.ihmc.simulationconstructionset;

import static us.ihmc.robotics.Assert.assertEquals;

import org.junit.jupiter.api.Disabled;
import org.junit.jupiter.api.Test;

import us.ihmc.commons.thread.ThreadTools;

public class SimulationRewoundListenerTest
{
   /**
    * Hang forever
    */
   @Disabled
   @Test // timeout=300000
   public void testSimulationRewoundListener()
   {
      boolean showGUI = false;

      SimpleSimulationRewoundListener simulationRewoundListener = new SimpleSimulationRewoundListener();

      Robot robot = new Robot("Test");
      SimulationConstructionSetParameters parameters = SimulationConstructionSetParameters.createFromSystemProperties();
      parameters.setCreateGUI(showGUI);
      SimulationConstructionSet scs = new SimulationConstructionSet(robot, parameters);
      scs.setDT(0.001, 10);

      scs.attachSimulationRewoundListener(simulationRewoundListener);
      scs.startOnAThread();

      assertEquals(0, simulationRewoundListener.getCount());
      scs.simulate(1.0);
      while (scs.isSimulating())
      {
         ThreadTools.sleep(10);
      }
      assertEquals(1, simulationRewoundListener.getCount());
      assertEquals(100, scs.getCurrentIndex());

      scs.gotoInPointNow();
      assertEquals(2, simulationRewoundListener.getCount());
      assertEquals(0, scs.getCurrentIndex());

      ThreadTools.sleep(100);
      scs.tickAndReadFromBuffer(1);
      assertEquals(3, simulationRewoundListener.getCount());
      assertEquals(1, scs.getCurrentIndex());

      scs.tickAndReadFromBuffer(5);
      assertEquals(4, simulationRewoundListener.getCount());
      assertEquals(6, scs.getCurrentIndex());

      scs.tickAndReadFromBuffer(-1);
      assertEquals(5, simulationRewoundListener.getCount());
      assertEquals(5, scs.getCurrentIndex());

      scs.tickAndUpdate();
      assertEquals(5, simulationRewoundListener.getCount());
      assertEquals(6, scs.getCurrentIndex());

      scs.gotoOutPointNow();
      assertEquals(6, simulationRewoundListener.getCount());
      assertEquals(6, scs.getCurrentIndex());

      scs.play();
      ThreadTools.sleep(1000);
      assertEquals(6, simulationRewoundListener.getCount());

      scs.closeAndDispose();
   }

   private class SimpleSimulationRewoundListener implements RewoundListener
   {
      private int count = 0;

      @Override
      public void notifyOfRewind()
      {
         //         System.out.println(count + ": Sim was rewound");
         count++;
      }

      public int getCount()
      {
         return count;
      }
   }
}
