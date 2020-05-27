package us.ihmc.simulationconstructionset;

import java.util.List;

/**
 * <p>
 * Title: SimulationConstructionSet
 * </p>
 * <p>
 * Description:
 * </p>
 *
 * @author not attributable
 * @version 1.0
 */
public class UnreasonableAccelerationException extends Throwable
{
   private static final long serialVersionUID = 3265459168572596454L;
   private final List<Joint> unreasonableAccelerationJoints;

   public UnreasonableAccelerationException()
   {
      unreasonableAccelerationJoints = null;
   }

   public UnreasonableAccelerationException(List<Joint> unreasonableAccelerationJoints)
   {
      this.unreasonableAccelerationJoints = unreasonableAccelerationJoints;
   }

   public List<Joint> getUnreasonableAccelerationJoints()
   {
      return unreasonableAccelerationJoints;
   }
}
