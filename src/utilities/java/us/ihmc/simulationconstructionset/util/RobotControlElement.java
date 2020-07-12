package us.ihmc.simulationconstructionset.util;

import us.ihmc.yoVariables.registry.YoRegistry;

public interface RobotControlElement
{
   public abstract void initialize();

   public abstract YoRegistry getYoRegistry();

   public default String getName()
   {
      return getClass().getSimpleName();
   }

   public default String getDescription()
   {
      return "Robot controller for " + getName() + ".";
   }
}
