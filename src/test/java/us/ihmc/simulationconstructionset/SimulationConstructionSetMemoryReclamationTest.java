package us.ihmc.simulationconstructionset;

import static us.ihmc.robotics.Assert.assertTrue;

import java.awt.Frame;
import java.io.File;

import org.junit.jupiter.api.Tag;
import org.junit.jupiter.api.Test;

import us.ihmc.simulationconstructionset.examples.FallingBrickRobot;
import us.ihmc.tools.MemoryTools;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoDouble;

@Tag("gui")
public class SimulationConstructionSetMemoryReclamationTest
{
   private static final boolean DEBUG = true;

   @Test // timeout = 51000
   public void testMemoryReclamationForSCSWithoutARobot()
   {
      boolean useRobot = false;
      int numberOfTests = 3;
      boolean createVideo = false;
      int usedMemoryMBAtStart = MemoryTools.getCurrentMemoryUsageInMB();
      int usedMemoryMBAtEnd = testOneAndReturnUsedMemoryMB(useRobot, numberOfTests, createVideo);
      int usedMemoryMB = usedMemoryMBAtEnd - usedMemoryMBAtStart;

      checkForLingeringFrames();
      assertTrue("usedMemoryMB = " + usedMemoryMB, usedMemoryMB < 100);
   }

   @Test // timeout = 32000
   public void testMemoryReclamationForSCSWithARobot()
   {
      boolean useRobot = true;
      int numberOfTests = 1;
      boolean createVideo = false;
      int usedMemoryMBAtStart = MemoryTools.getCurrentMemoryUsageInMB();
      int usedMemoryMBAtEnd = testOneAndReturnUsedMemoryMB(useRobot, numberOfTests, createVideo);
      int usedMemoryMB = usedMemoryMBAtEnd - usedMemoryMBAtStart;

      checkForLingeringFrames();
      assertTrue("usedMemoryMB = " + usedMemoryMB, usedMemoryMB < 100);
   }

   // TODO https://jira.ihmc.us/browse/DRC-2208
   @Test // timeout=300000
   public void testMemoryReclamationForSCSWithARobotAndVideo()
   {
      boolean useRobot = true;
      int numberOfTests = 10;
      boolean createVideo = true;
      int usedMemoryMBAtStart = MemoryTools.getCurrentMemoryUsageInMB();
      int usedMemoryMBAtEnd = testOneAndReturnUsedMemoryMB(useRobot, numberOfTests, createVideo);
      int usedMemoryMB = usedMemoryMBAtEnd - usedMemoryMBAtStart;

      checkForLingeringFrames();
      assertTrue("usedMemoryMB = " + usedMemoryMB, usedMemoryMB < 100);
   }

   private int testOneAndReturnUsedMemoryMB(boolean useARobot, int numberOfTests, boolean createVideo)
   {
      boolean garbageCollect = true;
      MemoryTools.printCurrentMemoryUsageAndReturnUsedMemoryInMB("testOneAndReturnUsedMemoryMB start:", DEBUG, garbageCollect);

      for (int i = 0; i < numberOfTests; i++)
      {
         SimulationConstructionSet scs = createAndStartSimulationConstructionSet(useARobot);
         scs.simulate(2.0);

         sleep(2000);
         if (createVideo)
         {
            scs.gotoInPointNow();

            String videoFilename = "testOneAndReturnUsedMemoryMB.mp4";
            File file = new File(videoFilename);
            if (file.exists())
               file.delete();

            File videoFile = scs.createVideo(videoFilename);
            videoFile.delete();

            printIfDebug("Got past video creation...maybe");
         }
         scs.closeAndDispose();
         MemoryTools.printCurrentMemoryUsageAndReturnUsedMemoryInMB("testOneAndReturnUsedMemoryMB final: ", DEBUG, garbageCollect);
      }
      System.gc();

      printIfDebug("Created and disposed of " + numberOfTests + " SCSs. Should be garbage collected now...");
      sleep(2000);
      int usedMemoryMB = MemoryTools.getCurrentMemoryUsageInMB();
      printIfDebug("Used Memory = " + usedMemoryMB + " MB");

      return usedMemoryMB;
   }

   private void checkForLingeringFrames()
   {
      Frame[] frames = Frame.getFrames();
      if (frames != null)
      {
         printIfDebug("Number of Frames is still " + frames.length);
         for (Frame frame : frames)
         {
            printIfDebug("Frame " + frame.getTitle() + ": " + frame);
         }
      }
      frames = null;
   }

   private void printIfDebug(String string)
   {
      if (DEBUG)
         System.out.println(string);
   }

   private SimulationConstructionSet createAndStartSimulationConstructionSet(boolean useARobot)
   {
      SimulationConstructionSet scs;

      if (useARobot)
      {
         FallingBrickRobot robot = new FallingBrickRobot();
         YoRegistry registry = new YoRegistry("TestRegistry");

         for (int i = 0; i < 5000; i++)
         {
            new YoDouble("variable" + i, registry);
         }

         robot.addYoRegistry(registry);

         SimulationConstructionSetParameters parameters = SimulationConstructionSetParameters.createFromSystemProperties();
         parameters.setDataBufferSize(5000);
         scs = new SimulationConstructionSet(robot, parameters);
      }
      else
      {
         SimulationConstructionSetParameters parameters = SimulationConstructionSetParameters.createFromSystemProperties();
         parameters.setCreateGUI(true);
         parameters.setDataBufferSize(5000);
         scs = new SimulationConstructionSet(parameters);
      }

      scs.setDT(0.0001, 100);

      Thread thread = new Thread(scs);
      thread.start();

      while (useARobot && !scs.isSimulationThreadRunning())
      {
         sleep(100);
      }

      return scs;
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
