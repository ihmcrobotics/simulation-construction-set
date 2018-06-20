package us.ihmc.simulationconstructionset.physics.collision.gdx;

import org.junit.Test;

import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationPlan;
import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationTest;
import us.ihmc.continuousIntegration.IntegrationCategory;
import us.ihmc.simulationconstructionset.physics.ScsCollisionDetector;
import us.ihmc.simulationconstructionset.physics.collision.SCSCollisionDetectorTest;

@ContinuousIntegrationPlan(categories = IntegrationCategory.EXCLUDE)
public class GdxCollisionDetectorTest extends SCSCollisionDetectorTest
{
   @Override
   public ScsCollisionDetector createCollisionDetector()
   {
      return new GdxCollisionDetector(1000.0);
   }

   @Override
   @Test(timeout = 300000)
   public void collisionMask_hit()
   {
      super.collisionMask_hit();
   }

   @Override
   @Test(timeout = 300000)
   public void testBoxBarelyCollisions()
   {
      super.testBoxBarelyCollisions();
   }

   @Override
   @Test(timeout = 300000)
   public void testBoxCloseButNoCollisions()
   {
      super.testBoxCloseButNoCollisions();
   }

   @Override
   @Test(timeout = 300000)
   public void testSmallBox()
   {
      super.testSmallBox();
   }

   @Override
   @Test(timeout = 300000)
   public void testUnitBox()
   {
      super.testUnitBox();
   }

   @Override
   @Test(timeout = 300000)
   public void checkCollisionShape_offset()
   {
      super.checkCollisionShape_offset();
   }
}
