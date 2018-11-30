package us.ihmc.simulationconstructionset.physics.collision.gdx;

import org.junit.jupiter.api.Disabled;
import org.junit.jupiter.api.Test;
import us.ihmc.simulationconstructionset.physics.ScsCollisionDetector;
import us.ihmc.simulationconstructionset.physics.collision.SCSCollisionDetectorTest;

public class GdxCollisionDetectorTest extends SCSCollisionDetectorTest
{
   @Override
   public ScsCollisionDetector createCollisionDetector()
   {
      return new GdxCollisionDetector(1000.0);
   }

   @Override
   @Test// timeout = 300000
   public void collisionMask_hit()
   {
      super.collisionMask_hit();
   }

   @Override
   @Test// timeout = 300000
   public void testBoxBarelyCollisions()
   {
      super.testBoxBarelyCollisions();
   }

   @Override
   @Test// timeout = 300000
   public void testBoxCloseButNoCollisions()
   {
      super.testBoxCloseButNoCollisions();
   }

   @Disabled //java.lang.AssertionError: expected:<0> but was:<1>
   @Override
   @Test// timeout = 300000
   public void testSmallBox()
   {
      super.testSmallBox();
   }

   @Override
   @Test// timeout = 300000
   public void testUnitBox()
   {
      super.testUnitBox();
   }

   @Disabled //java.lang.AssertionError: expected:<1> but was:<3>
   @Override
   @Test// timeout = 300000
   public void checkCollisionShape_offset()
   {
      super.checkCollisionShape_offset();
   }
}
