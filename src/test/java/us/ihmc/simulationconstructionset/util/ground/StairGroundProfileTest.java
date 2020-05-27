package us.ihmc.simulationconstructionset.util.ground;

import org.junit.jupiter.api.Test;

import us.ihmc.jMonkeyEngineToolkit.GroundProfile3D;

public class StairGroundProfileTest extends GroundProfileTest
{
   @Override
   @Test // timeout=300000
   public void testSurfaceNormalGridForSmoothTerrainUsingHeightMap()
   {
      super.testSurfaceNormalGridForSmoothTerrainUsingHeightMap();
   }

   @Override
   public GroundProfile3D getGroundProfile()
   {
      return new StairGroundProfile(0.3, 0.2);
   }

   @Override
   public double getMaxPercentageOfAllowableValleyPoints()
   {
      return 0.0;
   }

   @Override
   public double getMaxPercentageOfAllowablePeakPoints()
   {
      return 0.0;
   }

   @Override
   public double getMaxPercentageOfAllowableDropOffs()
   {
      return 0.03;
   }
}
