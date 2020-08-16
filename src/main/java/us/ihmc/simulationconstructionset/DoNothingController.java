package us.ihmc.simulationconstructionset;

import us.ihmc.simulationconstructionset.util.RobotController;
import us.ihmc.yoVariables.registry.YoRegistry;

public final class DoNothingController implements RobotController
{
   private final YoRegistry registry = new YoRegistry("DoNothingController");

   @Override
   public YoRegistry getYoRegistry()
   {
      return registry;
   }

   @Override
   public void doControl()
   {
   }

   @Override
   public String getName()
   {
      return "doNothing";
   }

   @Override
   public void initialize()
   {
   }

   @Override
   public String getDescription()
   {
      return getName();
   }
}