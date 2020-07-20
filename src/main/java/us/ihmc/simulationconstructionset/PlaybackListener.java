package us.ihmc.simulationconstructionset;

import us.ihmc.yoVariables.buffer.interfaces.YoBufferIndexChangedListener;

public interface PlaybackListener extends YoBufferIndexChangedListener
{
   public void play(double realTimeRate);

   public void stop();

   // public void setRealTimeRate(double realTimeRate);
}
