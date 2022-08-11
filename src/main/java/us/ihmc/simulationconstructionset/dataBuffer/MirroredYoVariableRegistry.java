package us.ihmc.simulationconstructionset.dataBuffer;

import java.util.List;
import java.util.concurrent.ConcurrentLinkedQueue;

import us.ihmc.yoVariables.listener.YoVariableChangedListener;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoVariable;

public class MirroredYoVariableRegistry extends YoRegistry
{
   private final ConcurrentLinkedQueue<Runnable> mirrorPendingActions = new ConcurrentLinkedQueue<>();
   private final ConcurrentLinkedQueue<Runnable> originalPendingActions = new ConcurrentLinkedQueue<>();

   private boolean enableChangedListener = true;

   public MirroredYoVariableRegistry(YoRegistry original)
   {
      super(original.getName());

      copyRegistry(original, this);
   }

   private void copyRegistry(YoRegistry original, YoRegistry mirror)
   {
      List<YoVariable> originalVariables = original.getVariables();

      for (YoVariable originalVariable : originalVariables)
      {
         YoVariable mirrorVariable = originalVariable.duplicate(mirror);
         mirrorVariable.addListener(new YoVariableChangeForwarder(mirrorVariable, originalVariable, mirrorPendingActions));
         originalVariable.addListener(new YoVariableChangeForwarder(originalVariable, mirrorVariable, originalPendingActions));
      }

      for (YoRegistry childMirror : original.getChildren())
      {
         YoRegistry newRegistry = new YoRegistry(childMirror.getName());
         mirror.addChild(newRegistry);
         copyRegistry(childMirror, newRegistry);
      }
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
      while (!mirrorPendingActions.isEmpty())
         mirrorPendingActions.poll().run();
   }

   /**
    * Mirrors changes from the original registry to the mirror registry
    */
   public void updateValuesFromOriginal()
   {
      while (!originalPendingActions.isEmpty())
         originalPendingActions.poll().run();
   }

   private class YoVariableChangeForwarder implements YoVariableChangedListener
   {
      private final Runnable forwardAction;
      private final ConcurrentLinkedQueue<Runnable> forwardActionQueue;

      public YoVariableChangeForwarder(YoVariable source, YoVariable target, ConcurrentLinkedQueue<Runnable> forwardActionQueue)
      {
         this.forwardActionQueue = forwardActionQueue;

         forwardAction = () ->
         {
            enableChangedListener = false;
            target.setValue(source, true);
            enableChangedListener = true;
         };
      }

      @Override
      public void changed(YoVariable source)
      {
         if (enableChangedListener)
            forwardActionQueue.add(forwardAction);
      }
   }
}