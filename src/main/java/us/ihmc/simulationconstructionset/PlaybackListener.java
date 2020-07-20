package us.ihmc.simulationconstructionset;

import us.ihmc.yoVariables.dataBuffer.BufferIndexChangedListener;

public interface PlaybackListener extends BufferIndexChangedListener
{
   public void play(double realTimeRate);

   public void stop();

   // public void setRealTimeRate(double realTimeRate);
}
