package us.ihmc.simulationconstructionset.physics.engine.featherstone;

import us.ihmc.euclid.matrix.interfaces.RotationMatrixBasics;
import us.ihmc.euclid.matrix.interfaces.RotationMatrixReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;
import us.ihmc.simulationconstructionset.BallAndSocketJoint;

public class BallAndSocketJointPhysics extends JointPhysics<BallAndSocketJoint>
{

   public BallAndSocketJointPhysics(BallAndSocketJoint owner)
   {
      super(owner);
   }

   @Override
   public void featherstonePassOne(Vector3DReadOnly w_h, Vector3DReadOnly v_h, RotationMatrixReadOnly Rh_0)
   {
      
   }

   @Override
   public void recursiveEulerIntegrate(double stepSize)
   {
//      throw new RuntimeException("Implement me!");
   }

   @Override
   public void recursiveSaveTempState()
   {
//      throw new RuntimeException("Implement me!");
   }

   @Override
   public void recursiveRestoreTempState()
   {
//      throw new RuntimeException("Implement me!");
   }

   @Override
   public void recursiveRungeKuttaSum(double stepSize)
   {
//      throw new RuntimeException("Implement me!");
   }

   @Override
   protected void jointDependentSetAndGetRotation(RotationMatrixBasics Rh_i)
   {
      //throw new RuntimeException("Implement me!");
   }

   @Override
   protected void jointDependentFeatherstonePassOne()
   {
//      throw new RuntimeException("Implement me!");
   }

   @Override
   protected void jointDependentSet_d_i()
   {
      //throw new RuntimeException("Implement me!");
   }

   @Override
   protected void jointDependentFeatherstonePassTwo(Vector3DReadOnly w_h)
   {
//      throw new RuntimeException("Implement me!");
   }

   @Override
   protected void jointDependentFeatherstonePassFour(double Q, int passNumber)
   {
//      throw new RuntimeException("Implement me!");
   }

   @Override
   protected void jointDependentRecordK(int passNumber)
   {
//      throw new RuntimeException("Implement me!");
   }

   @Override
   protected boolean jointDependentVerifyReasonableAccelerations()
   {
      return true;
//      throw new RuntimeException("Implement me!");
   }

   @Override
   protected void jointDependentChangeVelocity(double delta_qd)
   {
//      throw new RuntimeException("Implement me!");
   }

}
