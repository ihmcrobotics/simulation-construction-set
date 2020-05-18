package us.ihmc.simulationconstructionset.util.ground;

import org.junit.jupiter.api.Test;

import us.ihmc.jMonkeyEngineToolkit.GroundProfile3D;

public class WavyGroundProfileTest extends GroundProfileTest
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
      return new WavyGroundProfile();
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
      //TODO: 30 percent is too high. Need to actually compute the surface normal properly...
      return 0.3;
   }
}
