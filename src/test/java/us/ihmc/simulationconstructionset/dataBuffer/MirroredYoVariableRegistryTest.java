package us.ihmc.simulationconstructionset.dataBuffer;

import static us.ihmc.robotics.Assert.assertEquals;
import static us.ihmc.robotics.Assert.assertFalse;
import static us.ihmc.robotics.Assert.assertTrue;

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