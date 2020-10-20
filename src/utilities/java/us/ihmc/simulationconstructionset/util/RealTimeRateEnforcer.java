package us.ihmc.simulationconstructionset.util;

import us.ihmc.commons.thread.Notification;
import us.ihmc.commons.thread.ThreadTools;

import java.util.concurrent.ScheduledExecutorService;
import java.util.concurrent.TimeUnit;

/**
 * Class for enforcing that a simulation or other computational process doesn't run any faster than
 * real time. Of course, if it runs slower than real time, nothing you can do about it except speed
 * up the algorithm or buy a faster computer. Make sure to call reset() whenever the simulated clock
 * and the wall clock become out of synch (for example, stopping or rewinding a simulation).
 */
public class RealTimeRateEnforcer
{
   private long wallStartTimeInMilliseconds = -1;
   private double simulatedStartTimeInSeconds = -1.0;
   private final ScheduledExecutorService wakupScheduler = ThreadTools.newSingleThreadScheduledExecutor("RealTimeRateEnforcer");
   private final Notification sleepNotification = new Notification();

   public void sleepIfNecessaryToEnforceRealTimeRate(double simulatedCurrentTimeInSeconds)
   {
      if (wallStartTimeInMilliseconds == -1)
      {
         wallStartTimeInMilliseconds = System.currentTimeMillis();
         simulatedStartTimeInSeconds = simulatedCurrentTimeInSeconds;
         return;
      }

      long wallCurrentTimeInMilliseconds = System.currentTimeMillis();

      int simulatedElapsedTimeInMilliseconds = (int) (1000.0 * (simulatedCurrentTimeInSeconds - simulatedStartTimeInSeconds));
      long wallElapsedTimeInMilliseconds = wallCurrentTimeInMilliseconds - wallStartTimeInMilliseconds;

      int timeToSleepInMilliseconds = (int) (simulatedElapsedTimeInMilliseconds - wallElapsedTimeInMilliseconds);
      if (timeToSleepInMilliseconds > 10)
      {
         wakupScheduler.schedule(sleepNotification::set, timeToSleepInMilliseconds, TimeUnit.MILLISECONDS);
         sleepNotification.blockingPoll();
      }
   }

   public void reset()
   {
      wallStartTimeInMilliseconds = -1;
      simulatedStartTimeInSeconds = -1.0;
   }
}
