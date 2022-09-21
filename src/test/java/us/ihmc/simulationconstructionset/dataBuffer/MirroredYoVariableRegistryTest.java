package us.ihmc.simulationconstructionset.dataBuffer;

import static us.ihmc.robotics.Assert.assertEquals;
import static us.ihmc.robotics.Assert.assertFalse;
import static us.ihmc.robotics.Assert.assertTrue;

import java.util.List;

import org.apache.commons.lang3.StringUtils;
import org.junit.jupiter.api.Test;

import us.ihmc.yoVariables.listener.YoVariableChangedListener;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoVariable;

public class MirroredYoVariableRegistryTest
{
   private static final int TEST_VARIABLE_COUNT = 10;

   @Test // timeout = 30000
   public void testMirroredRegistryIsTheSameAsOriginalAfterCreation()
   {
      YoRegistry originalRegistry = createTestRegistry("OriginalRegistry", TEST_VARIABLE_COUNT);

      MirroredYoVariableRegistry mirroredRegistry = new MirroredYoVariableRegistry(originalRegistry);

      assertTrue(areRegistryVariablesAreEqual(originalRegistry, mirroredRegistry));
   }

   @Test // timeout = 30000
   public void testYoRegistryChildren()
   {
      YoRegistry originalRegistry = createTestRegistry("OriginalRegistry", TEST_VARIABLE_COUNT);
      YoRegistry childRegistry = createTestRegistry("ChildRegistry", TEST_VARIABLE_COUNT);

      originalRegistry.addChild(childRegistry);

      MirroredYoVariableRegistry mirroredRegistry = new MirroredYoVariableRegistry(originalRegistry);

      for (int i = 0; i < mirroredRegistry.getChildren().size(); i++)
      {
         assertTrue(areRegistryVariablesAreEqual(originalRegistry.getChildren().get(i), mirroredRegistry.getChildren().get(i)));
      }

   }

   @Test // timeout = 30000
   public void testChangesArePropagatedFromOriginal()
   {
      YoRegistry originalRegistry = createTestRegistry("OriginalRegistry", TEST_VARIABLE_COUNT);
      MirroredYoVariableRegistry mirroredYoRegistry = new MirroredYoVariableRegistry(originalRegistry);

      for (YoVariable yoVariable : originalRegistry.collectSubtreeVariables())
      {
         yoVariable.setValueFromDouble(1.0);
      }

      // Should *not* be equal until updateValuesFromOriginal or updateMirror is called
      assertFalse(areRegistryVariablesAreEqual(originalRegistry, mirroredYoRegistry));

      mirroredYoRegistry.updateValuesFromOriginal();

      assertTrue(areRegistryVariablesAreEqual(originalRegistry, mirroredYoRegistry));
   }

   @Test // timeout = 30000
   public void testChangesArePropagatedFromMirror()
   {
      YoRegistry originalRegistry = createTestRegistry("OriginalRegistry", TEST_VARIABLE_COUNT);
      MirroredYoVariableRegistry mirroredYoRegistry = new MirroredYoVariableRegistry(originalRegistry);

      for (YoVariable yoVariable : mirroredYoRegistry.collectSubtreeVariables())
      {
         yoVariable.setValueFromDouble(2.0);
      }

      // Should *not* be equal until updateChangedValues or updateMirror is called
      assertFalse(areRegistryVariablesAreEqual(originalRegistry, mirroredYoRegistry));

      mirroredYoRegistry.updateChangedValues();

      assertTrue(areRegistryVariablesAreEqual(originalRegistry, mirroredYoRegistry));
   }

   @Test
   public void testListenerBug20220829()
   {
      /*
       * The bug was essentially ignoring changes in the original variables when they were applied from
       * inside a YoVariableChangedListener.
       */
      YoRegistry originalRegistry = createTestRegistry("OriginalRegistry", TEST_VARIABLE_COUNT);
      MirroredYoVariableRegistry mirroredRegistry = new MirroredYoVariableRegistry(originalRegistry);

      List<YoVariable> originalVariables = originalRegistry.collectSubtreeVariables();
      YoVariable originalV0 = originalVariables.get(0);
      YoVariable originalV1 = originalVariables.get(1);

      List<YoVariable> mirroredVariables = mirroredRegistry.collectSubtreeVariables();
      YoVariable mirroredV0 = mirroredVariables.get(0);
      YoVariable mirroredV1 = mirroredVariables.get(1);

      // Linking originalV1 to originalV0
      originalV0.addListener(v -> originalV1.setValueFromDouble(3.0));

      assertTrue(areRegistryVariablesAreEqual(originalRegistry, mirroredRegistry));

      mirroredV0.setValueFromDouble(1.0);

      // This should cause the following actions:
      // 1- originalV0 is set to mirroredV0 value (1.0)
      // 2- originalV0's changes triggers its own listener and update originalV1
      // 3- mirroredV1 is set to originalV1 value (3.0)
      // The bug was causing to ignore that originalV1 changed and thus essentially skip step 3.
      mirroredRegistry.updateMirror();

      assertTrue(areYoVariablesEqual(originalV0, mirroredV0));
      assertTrue(areYoVariablesEqual(originalV1, mirroredV1));

      // Asserts that after 1 update we are done, i.e. there is no more pending actions
      assertTrue(mirroredRegistry.getMirrorPendingActions().isEmpty());
      assertTrue(mirroredRegistry.getOriginalPendingActions().isEmpty());

      assertTrue(areRegistryVariablesAreEqual(originalRegistry, mirroredRegistry));
   }

   @Test // timeout = 30000
   public void testMirrorValuesArePreferredWhenConflict()
   {
      YoRegistry originalRegistry = createTestRegistry("OriginalRegistry", TEST_VARIABLE_COUNT);
      MirroredYoVariableRegistry mirroredYoRegistry = new MirroredYoVariableRegistry(originalRegistry);

      final double newValueForOriginal = 2.0;
      final double newValueForMirror = 3.0;

      for (YoVariable yoVariable : originalRegistry.collectSubtreeVariables())
      {
         yoVariable.setValueFromDouble(newValueForOriginal);
      }

      for (YoVariable yoVariable : mirroredYoRegistry.collectSubtreeVariables())
      {
         yoVariable.setValueFromDouble(newValueForMirror);
      }

      // Should *not* be equal until updateMirror is called
      assertFalse(areRegistryVariablesAreEqual(originalRegistry, mirroredYoRegistry));

      mirroredYoRegistry.updateMirror();

      assertTrue(areRegistryVariablesAreEqual(originalRegistry, mirroredYoRegistry));

      for (YoVariable yoVariable : mirroredYoRegistry.collectSubtreeVariables())
      {
         assertEquals(yoVariable.getValueAsDouble(), newValueForMirror, 1e-10);
      }
   }

   @Test // timeout = 30000
   public void testOriginalListenersAreCalledWhenMirrorChanges()
   {
      YoRegistry originalRegistry = createTestRegistry("OriginalRegistry", TEST_VARIABLE_COUNT);
      MirroredYoVariableRegistry mirroredYoRegistry = new MirroredYoVariableRegistry(originalRegistry);

      ListenerCounter listenerCounter = new ListenerCounter();
      for (YoVariable yoVariable : originalRegistry.collectSubtreeVariables())
      {
         yoVariable.addListener(listenerCounter);
      }

      for (YoVariable yoVariable : mirroredYoRegistry.collectSubtreeVariables())
      {
         yoVariable.setValueFromDouble(1.0);
      }

      mirroredYoRegistry.updateMirror();

      assertEquals(listenerCounter.callCount, originalRegistry.collectSubtreeVariables().size());
   }

   @Test // timeout = 30000
   public void testMirrorListenersAreCalledWhenOriginalChanges()
   {
      YoRegistry originalRegistry = createTestRegistry("OriginalRegistry", TEST_VARIABLE_COUNT);
      MirroredYoVariableRegistry mirroredYoRegistry = new MirroredYoVariableRegistry(originalRegistry);

      ListenerCounter listenerCounter = new ListenerCounter();
      for (YoVariable yoVariable : mirroredYoRegistry.collectSubtreeVariables())
      {
         yoVariable.addListener(listenerCounter);
      }

      for (YoVariable yoVariable : originalRegistry.collectSubtreeVariables())
      {
         yoVariable.setValueFromDouble(1.0);
      }

      mirroredYoRegistry.updateMirror();

      assertEquals(listenerCounter.callCount, mirroredYoRegistry.collectSubtreeVariables().size());
   }

   @Test
   public void testListenerBug20220921()
   {
      YoRegistry originalRegistry = createTestRegistry("OriginalRegistry", TEST_VARIABLE_COUNT);
      MirroredYoVariableRegistry mirroredRegistry = new MirroredYoVariableRegistry(originalRegistry);
      List<YoVariable> mirroredVariables = mirroredRegistry.collectSubtreeVariables();
      List<YoVariable> originalVariables = originalRegistry.collectSubtreeVariables();

      for (YoVariable mirroredVariable : mirroredVariables)
      {
         mirroredVariable.setValueFromDouble(2.0);
      }

      YoVariable aOriginalVariable = originalVariables.get(originalVariables.size() / 2);
      aOriginalVariable.setValueFromDouble(5.0);
      YoVariableChangedListener aOriginalVariableListener = v ->
      {
         if (aOriginalVariable.getValueAsDouble() != 5.0)
            aOriginalVariable.setValueFromDouble(5.0);
      };
      aOriginalVariable.addListener(aOriginalVariableListener);
      YoVariable aMirroredVariable = mirroredRegistry.findVariable(aOriginalVariable.getFullNameString());

      assertFalse(areRegistryVariablesAreEqual(originalRegistry, mirroredRegistry));

      mirroredRegistry.updateChangedValues();

      // The aOriginalVariable's value should remain 5.0
      assertEquals(5.0, aOriginalVariable.getValueAsDouble());
      assertEquals(2.0, aMirroredVariable.getValueAsDouble()); // The original value has not been back propagated yet
      assertFalse(areRegistryVariablesAreEqual(originalRegistry, mirroredRegistry));

      mirroredRegistry.updateValuesFromOriginal();
      assertEquals(5.0, aOriginalVariable.getValueAsDouble());
      assertEquals(5.0, aMirroredVariable.getValueAsDouble());
      assertTrue(areRegistryVariablesAreEqual(originalRegistry, mirroredRegistry));

      aMirroredVariable.setValueFromDouble(2.0);

      mirroredRegistry.updateChangedValues();

      // The aOriginalVariable's value should remain 5.0
      assertEquals(5.0, aOriginalVariable.getValueAsDouble());
      assertEquals(2.0, aMirroredVariable.getValueAsDouble()); // The original value has not been back propagated yet
      assertFalse(areRegistryVariablesAreEqual(originalRegistry, mirroredRegistry));

      mirroredRegistry.updateValuesFromOriginal();
      assertEquals(5.0, aOriginalVariable.getValueAsDouble());
      assertEquals(5.0, aMirroredVariable.getValueAsDouble()); // The bug would show up now, the mirrored value would still be 2, and the original value 5
      assertTrue(areRegistryVariablesAreEqual(originalRegistry, mirroredRegistry));
   }

   private static YoRegistry createTestRegistry(String name, int variableCount)
   {
      YoRegistry registry = new YoRegistry(name);
      for (int i = 0; i < variableCount; i++)
      {
         new YoDouble("Variable" + i, registry);
      }
      return registry;
   }

   private static boolean areRegistryVariablesAreEqual(YoRegistry registry1, YoRegistry registry2)
   {
      if (registry1.collectSubtreeVariables().size() != registry2.collectSubtreeVariables().size())
         return false;

      for (int i = 0; i < registry1.collectSubtreeVariables().size(); i++)
      {
         YoVariable original = registry1.collectSubtreeVariables().get(i);
         YoVariable copy = registry2.collectSubtreeVariables().get(i);
         if (!areYoVariablesEqual(original, copy))
         {
            return false;
         }
      }
      return true;
   }

   private static boolean areYoVariablesEqual(YoVariable var1, YoVariable var2)
   {
      return StringUtils.equals(var1.getName(), var2.getName()) && var1.getValueAsDouble() == var2.getValueAsDouble();
   }

   private static class ListenerCounter implements YoVariableChangedListener
   {
      public int callCount = 0;

      @Override
      public void changed(YoVariable v)
      {
         ++callCount;
      }
   }
}