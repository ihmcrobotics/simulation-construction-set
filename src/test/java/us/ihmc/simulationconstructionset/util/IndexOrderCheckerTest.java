package us.ihmc.simulationconstructionset.util;

import org.junit.Before;
import org.junit.Test;
import us.ihmc.yoVariables.registry.YoVariableRegistry;

import static org.junit.Assert.assertEquals;


public class IndexOrderCheckerTest
{
   private IndexOrderChecker indexOrderChecker;

   @Before
   public void setUp()
   {
      YoVariableRegistry registry = new YoVariableRegistry("testRegistry");
      indexOrderChecker = new IndexOrderChecker("test", registry, 1);
   }

	@Test(timeout=300000)
   public void testNoOverflow()
   {
      int index = 0;
      indexOrderChecker.update(index);
      int increment = 5;
      index += increment;
      indexOrderChecker.update(index);
      assertEquals(increment - 1, indexOrderChecker.getMissedIndices());
   }

	@Test(timeout=300000)
   public void testOverflowOne()
   {
      int index = Integer.MAX_VALUE;
      indexOrderChecker.update(index);
      index++;
      indexOrderChecker.update(index);
      assertEquals(0, indexOrderChecker.getMissedIndices());
   }

	@Test(timeout=300000)
   public void testOverflowTwo()
   {
      int index = Integer.MAX_VALUE - 2;
      indexOrderChecker.update(index);
      int increment = 5;
      index += increment;
      indexOrderChecker.update(index);
      assertEquals(increment - 1, indexOrderChecker.getMissedIndices());
   }

	@Test(timeout=300000)
   public void testABunch()
   {
      assertEquals(0, indexOrderChecker.getMissedIndices());
      indexOrderChecker.update(0);
      assertEquals(0, indexOrderChecker.getMissedIndices());
      indexOrderChecker.update(1);
      assertEquals(0, indexOrderChecker.getMissedIndices());
      indexOrderChecker.update(3);
      assertEquals(1, indexOrderChecker.getMissedIndices());
      indexOrderChecker.update(8);
      assertEquals(5, indexOrderChecker.getMissedIndices());
   }
}
