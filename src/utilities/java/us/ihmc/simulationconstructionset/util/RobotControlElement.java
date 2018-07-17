package us.ihmc.simulationconstructionset.util;

import us.ihmc.yoVariables.registry.YoVariableRegistry;

public interface RobotControlElement
{
   public abstract void initialize();

   public abstract YoVariableRegistry getYoVariableRegistry();

   public default String getName()
   {
      return getClass().getSimpleName();
   }

   public default String getDescription()
   {
      return "Robot controller for " + getName() + ".";
   }
}
