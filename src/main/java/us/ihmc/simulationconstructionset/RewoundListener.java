package us.ihmc.simulationconstructionset;

/**
 * Listener on the progression of a {@link DataBuffer} index.
 */
public interface RewoundListener
{
   /**
    * Called when the {@link DataBuffer#index} is changed using its setter. This method is not be
    * called when the index ticks forward naturally.
    */
   public void notifyOfRewind();
}
