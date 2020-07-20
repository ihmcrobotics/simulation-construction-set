package us.ihmc.simulationconstructionset;

import java.util.ArrayList;
import java.util.List;

import us.ihmc.yoVariables.dataBuffer.YoBufferIndexChangedListener;

public class RewoundListenerHandler implements YoBufferIndexChangedListener
{
   private boolean enable = true;
   private final List<RewoundListener> listeners = new ArrayList<>();

   public void setEnable(boolean enable)
   {
      this.enable = enable;
   }

   public void addListener(RewoundListener listener)
   {
      listeners.add(listener);
   }

   public boolean removeListener(RewoundListener listener)
   {
      return listeners.remove(listener);
   }

   public void notifyListeners()
   {
      for (RewoundListener listener : listeners)
      {
         listener.notifyOfRewind();
      }
   }

   @Override
   public void notifyOfIndexChange(int newIndex)
   {
      if (!enable)
         return;

      notifyListeners();
   }
}
