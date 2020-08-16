package us.ihmc.simulationconstructionset.dataBuffer;

import java.util.List;
import java.util.concurrent.ConcurrentLinkedQueue;

import com.google.common.collect.BiMap;
import com.google.common.collect.HashBiMap;

import us.ihmc.yoVariables.listener.YoVariableChangedListener;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoVariable;

public class MirroredYoVariableRegistry extends YoRegistry
{
   private final BiMap<YoVariable, YoVariable> variableMap = HashBiMap.create();
   private final ConcurrentLinkedQueue<YoVariable> changedVariablesInMirror = new ConcurrentLinkedQueue<>();
   private final ConcurrentLinkedQueue<YoVariable> changedVariablesInOriginal = new ConcurrentLinkedQueue<>();

   private boolean enableChangedListener = true;
   private final YoRegistryChangedListener mirroredChangeListener = new YoRegistryChangedListener(changedVariablesInMirror);
   private final YoRegistryChangedListener originalChangeListener = new YoRegistryChangedListener(changedVariablesInOriginal);

   public MirroredYoVariableRegistry(YoRegistry original)
   {
      super(original.getName());

      copyRegistry(original, this);
   }

   private void copyRegistry(YoRegistry original, YoRegistry target)
   {
      List<YoVariable> vars = original.getVariables();

      for (YoVariable var : vars)
      {
         YoVariable newVar = var.duplicate(target);
         variableMap.put(var, newVar);
         addVariableListener(newVar, var);
      }

      for (YoRegistry child : original.getChildren())
      {
         YoRegistry newRegistry = new YoRegistry(child.getName());
         target.addChild(newRegistry);
         copyRegistry(child, newRegistry);
      }
   }

   private void addVariableListener(YoVariable newVar, YoVariable var)
   {
      newVar.addListener(mirroredChangeListener);
      var.addListener(originalChangeListener);
   }

   /**
    * Updates changes from the mirror to the original registry and then from the original to the mirror
    * registry
    */
   public void updateMirror()
   {
      updateChangedValues();
      updateValuesFromOriginal();
   }

   /**
    * Mirrors changes from the mirror registry to the original registry
    */
   public void updateChangedValues()
   {
      for (YoVariable changed = changedVariablesInMirror.poll(); changed != null; changed = changedVariablesInMirror.poll())
      {
         YoVariable originalVar = variableMap.inverse().get(changed);
         originalVar.setValue(changed, false);
         callListenersForVariable(originalVar);
      }
   }

   /**
    * Mirrors changes from the original registry to the mirror registry
    */
   public void updateValuesFromOriginal()
   {
      for (YoVariable changed = changedVariablesInOriginal.poll(); changed != null; changed = changedVariablesInOriginal.poll())
      {
         YoVariable mirroredVar = variableMap.get(changed);
         mirroredVar.setValue(changed, false);
         callListenersForVariable(mirroredVar);
      }
   }

   private void callListenersForVariable(YoVariable variable)
   {
      enableChangedListener = false;
      variable.notifyListeners();
      enableChangedListener = true;
   }

   private class YoRegistryChangedListener implements YoVariableChangedListener
   {
      final ConcurrentLinkedQueue<YoVariable> queue;

      private YoRegistryChangedListener(ConcurrentLinkedQueue<YoVariable> queue)
      {
         this.queue = queue;
      }

      @Override
      public void changed(YoVariable v)
      {
         if (enableChangedListener)
            queue.add(v);
      }
   }
}
