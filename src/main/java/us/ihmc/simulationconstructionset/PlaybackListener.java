package us.ihmc.simulationconstructionset;

import us.ihmc.yoVariables.dataBuffer.YoBufferIndexChangedListener;

public interface PlaybackListener extends YoBufferIndexChangedListener
{
   public void play(double realTimeRate);

   public void stop();

   // public void setRealTimeRate(double realTimeRate);
}
