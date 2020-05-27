package us.ihmc.simulationconstructionset.util.ground;

import static us.ihmc.robotics.Assert.assertEquals;

import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

import us.ihmc.tools.MemoryTools;

public class VaryingStairGroundProfileTest
{
   @BeforeEach
   public void showMemoryUsageBeforeTest()
   {
      MemoryTools.printCurrentMemoryUsageAndReturnUsedMemoryInMB(getClass().getSimpleName() + " before test.");
   }

   @AfterEach
   public void showMemoryUsageAfterTest()
   {
      MemoryTools.printCurrentMemoryUsageAndReturnUsedMemoryInMB(getClass().getSimpleName() + " after test.");
   }

   @Test // timeout = 30000
   public void test()
   {
      double startX = -0.1;
      double startZ = 3.0;
      double[] stepRises = new double[] {0.1, 0.2, 0.3};
      double[] stepTreads = new double[] {0.4, 0.5};
      VaryingStairGroundProfile groundProfile = new VaryingStairGroundProfile(startX, startZ, stepTreads, stepRises);

      double epsilonX = 1e-3;
      double epsilonZ = 1e-14;

      int nSteps = stepRises.length;

      double currentX = startX;
      double currentHeight = startZ;
      for (int i = 0; i < nSteps; i++)
      {
         assertEquals(currentHeight, groundProfile.heightAt(currentX - epsilonX, 0.0, 0.0), epsilonZ);
         currentHeight += stepRises[i];
         assertEquals(currentHeight, groundProfile.heightAt(currentX + epsilonX, 0.0, 0.0), epsilonZ);
         if (i < stepTreads.length)
            currentX += stepTreads[i];
      }

      System.out.println(currentX);
   }
}
