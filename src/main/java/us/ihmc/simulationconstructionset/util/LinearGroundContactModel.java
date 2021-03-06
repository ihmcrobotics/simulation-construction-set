package us.ihmc.simulationconstructionset.util;

import us.ihmc.simulationconstructionset.Robot;
import us.ihmc.yoVariables.registry.YoRegistry;

// Same as LinearStickSlipGroundContactModel, but if you use these constructors, it disables slipping.
public class LinearGroundContactModel extends LinearStickSlipGroundContactModel
{
   private static final long serialVersionUID = 3967257851444622051L;

   public LinearGroundContactModel(Robot rob, YoRegistry parentRegistry)
   {
      super(rob, parentRegistry);
      super.disableSlipping();
   }

   public LinearGroundContactModel(Robot rob, double groundKxy, double groundBxy, double groundKz, double groundBz, YoRegistry parentRegistry)
   {
      this(rob, 0, groundKxy, groundBxy, groundKz, groundBz, parentRegistry);
      super.disableSlipping();
   }

   public LinearGroundContactModel(Robot rob, int groundContactGroupIdentifier, double groundKxy, double groundBxy, double groundKz, double groundBz,
                                   YoRegistry parentRegistry)
   {
      super(rob, groundContactGroupIdentifier, groundKxy, groundBxy, groundKz, groundBz, parentRegistry);
      super.disableSlipping();
   }
}
