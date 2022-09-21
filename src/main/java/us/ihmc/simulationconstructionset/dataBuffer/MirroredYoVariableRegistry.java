package us.ihmc.simulationconstructionset.dataBuffer;

import java.util.List;
import java.util.concurrent.ConcurrentLinkedQueue;

import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoVariable;

public class MirroredYoVariableRegistry extends YoRegistry
{
   private final ConcurrentLinkedQueue<Runnable> mirrorPendingActions = new ConcurrentLinkedQueue<>();
   private final ConcurrentLinkedQueue<Runnable> originalPendingActions = new ConcurrentLinkedQueue<>();

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
         bindVariables(originalVariable, mirrorVariable, originalPendingActions, mirrorPendingActions);
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

   private static void bindVariables(YoVariable variableA,
                                     YoVariable variableB,
                                     ConcurrentLinkedQueue<Runnable> actionQueueA,
                                     ConcurrentLinkedQueue<Runnable> actionQueueB)
   {

      Runnable actionA = () -> variableB.setValue(variableA, true);
      Runnable actionB = () -> variableA.setValue(variableB, true);

      variableA.addListener(v ->
      {
         if (variableA.getValueAsLongBits() != variableB.getValueAsLongBits())
            actionQueueA.add(actionA);
      });
      variableB.addListener(v ->
      {
         if (variableA.getValueAsLongBits() != variableB.getValueAsLongBits())
            actionQueueB.add(actionB);
      });
   }

   ConcurrentLinkedQueue<Runnable> getMirrorPendingActions()
   {
      return mirrorPendingActions;
   }

   ConcurrentLinkedQueue<Runnable> getOriginalPendingActions()
   {
      return originalPendingActions;
   }
}