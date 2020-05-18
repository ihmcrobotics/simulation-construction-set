package us.ihmc.simulationconstructionset.util.ground;

import static us.ihmc.robotics.Assert.assertEquals;

import org.junit.jupiter.api.Assertions;
import org.junit.jupiter.api.Test;

import us.ihmc.jMonkeyEngineToolkit.GroundProfile3D;

public class AlternatingSlopesGroundProfileTest extends GroundProfileTest
{
   private static final double epsilon = 1e-7;

   @Override
   public GroundProfile3D getGroundProfile()
   {
      double[][] xSlopePairs = new double[][] {{-5.0, 1.0}, {0.0, -1.0}, {5.0, 1.0}};
      AlternatingSlopesGroundProfile groundProfile = new AlternatingSlopesGroundProfile(xSlopePairs);
      return groundProfile;
   }

   @Override
   public double getMaxPercentageOfAllowableValleyPoints()
   {
      return 0.05;
   }

   @Override
   public double getMaxPercentageOfAllowablePeakPoints()
   {
      return 0.05;
   }

   @Override
   public double getMaxPercentageOfAllowableDropOffs()
   {
      return 0.0;
   }

   @Override
   @Test // timeout=300000
   public void testSurfaceNormalGridForSmoothTerrainUsingHeightMap()
   {
      super.testSurfaceNormalGridForSmoothTerrainUsingHeightMap();
   }

   @Test // timeout=300000
   public void testAllFlat()
   {
      double[][] xSlopePairs = new double[][] {{0.0, 0.0}};
      AlternatingSlopesGroundProfile profile = new AlternatingSlopesGroundProfile(xSlopePairs);

      double height = profile.heightAt(-100.0, 0.0, 0.0);
      assertEquals(0.0, height, epsilon);

      height = profile.heightAt(0.0, 0.0, 0.0);
      assertEquals(0.0, height, epsilon);

      height = profile.heightAt(1000.0, 0.0, 0.0);
      assertEquals(0.0, height, epsilon);
   }

   @Test // timeout=300000
   public void testOne()
   {
      double[][] xSlopePairs = new double[][] {{1.0, 1.0}};
      AlternatingSlopesGroundProfile profile = new AlternatingSlopesGroundProfile(xSlopePairs);

      double[][] xzPairs = profile.getXZPairs();

      assertEquals(xzPairs.length, 3);
      assertEquals(-10.0, xzPairs[0][0], epsilon);
      assertEquals(0.0, xzPairs[0][1], epsilon);
      assertEquals(1.0, xzPairs[1][0], epsilon);
      assertEquals(0.0, xzPairs[1][1], epsilon);
      assertEquals(10.0, xzPairs[2][0], epsilon);
      assertEquals(9.0, xzPairs[2][1], epsilon);

      double height = profile.heightAt(0.0, 0.0, 0.0);
      assertEquals(0.0, height, epsilon);

      height = profile.heightAt(1.0, 0.0, 0.0);
      assertEquals(0.0, height, epsilon);

      height = profile.heightAt(1.5, 0.0, 0.0);
      assertEquals(0.5, height, epsilon);

      height = profile.heightAt(10.0, 0.0, 0.0);
      assertEquals(9.0, height, epsilon);

      height = profile.heightAt(100.0, 0.0, 0.0);
      assertEquals(9.0, height, epsilon);

      height = profile.heightAt(-100.0, 0.0, 0.0);
      assertEquals(0.0, height, epsilon);
   }

   @Test // timeout=300000
   public void testTwo()
   {
      double[][] xSlopePairs = new double[][] {{0.0, 0.0}, {1.0, 1.0}, {2.0, 0.0}};
      AlternatingSlopesGroundProfile profile = new AlternatingSlopesGroundProfile(xSlopePairs);

      double height = profile.heightAt(0.0, 0.0, 0.0);
      assertEquals(0.0, height, epsilon);

      height = profile.heightAt(1.0, 0.0, 0.0);
      assertEquals(0.0, height, epsilon);

      height = profile.heightAt(1.5, 0.0, 0.0);
      assertEquals(0.5, height, epsilon);

      height = profile.heightAt(2.0, 0.0, 0.0);
      assertEquals(1.0, height, epsilon);

      height = profile.heightAt(3.0, 0.0, 0.0);
      assertEquals(1.0, height, epsilon);
   }

   @Test // timeout=300000
   public void testThree()
   {
      double[][] xSlopePairs = new double[][] {{-5.0, 1.0}, {0.0, -1.0}, {5.0, 1.0}};
      AlternatingSlopesGroundProfile profile = new AlternatingSlopesGroundProfile(xSlopePairs);

      double height = profile.heightAt(-6.0, 0.0, 0.0);
      assertEquals(0.0, height, epsilon);

      height = profile.heightAt(-5.0, 0.0, 0.0);
      assertEquals(0.0, height, epsilon);

      height = profile.heightAt(-4.0, 0.0, 0.0);
      assertEquals(1.0, height, epsilon);

      height = profile.heightAt(0.0, 0.0, 0.0);
      assertEquals(5.0, height, epsilon);

      height = profile.heightAt(1.0, 0.0, 0.0);
      assertEquals(4.0, height, epsilon);

      height = profile.heightAt(5.0, 0.0, 0.0);
      assertEquals(0.0, height, epsilon);

      height = profile.heightAt(6.0, 0.0, 0.0);
      assertEquals(1.0, height, epsilon);
   }

   @Test // timeout=300000
   public void testFour()
   {
      double xMin = -100.0, xMax = 100.0, yMin = -100.0, yMax = 100.0;

      double[][] xSlopePairs = new double[][] {{-5.0, 1.0}, {0.0, -1.0}, {5.0, 1.0}};
      AlternatingSlopesGroundProfile profile = new AlternatingSlopesGroundProfile(xSlopePairs, xMin, xMax, yMin, yMax);

      double height = profile.heightAt(-6.0, 0.0, 0.0);
      assertEquals(0.0, height, epsilon);

      height = profile.heightAt(-5.0, 0.0, 0.0);
      assertEquals(0.0, height, epsilon);

      height = profile.heightAt(-4.0, 0.0, 0.0);
      assertEquals(1.0, height, epsilon);

      height = profile.heightAt(0.0, 0.0, 0.0);
      assertEquals(5.0, height, epsilon);

      height = profile.heightAt(1.0, 0.0, 0.0);
      assertEquals(4.0, height, epsilon);

      height = profile.heightAt(5.0, 0.0, 0.0);
      assertEquals(0.0, height, epsilon);

      height = profile.heightAt(6.0, 0.0, 0.0);
      assertEquals(1.0, height, epsilon);
   }

   @Test // expected = RuntimeException.class,timeout=300000
   public void testBadOrderingOne()
   {
      Assertions.assertThrows(RuntimeException.class, () ->
      {
         double[][] xSlopePairs = new double[][] {{0.0, -1.0}, {1e-10, 1.0}};
         new AlternatingSlopesGroundProfile(xSlopePairs);
      });
   }

   @Test // expected = RuntimeException.class, timeout=300000
   public void testBadOrderingTwo()
   {
      Assertions.assertThrows(RuntimeException.class, () ->
      {
         double[][] xSlopePairs = new double[][] {{0.0, -1.0}, {1.0, 1.0}, {3.5, 1.0}, {3.0, 1.0}, {4.0, 1.0}};
         new AlternatingSlopesGroundProfile(xSlopePairs);
      });
   }
}
