package us.ihmc.simulationconstructionset;

import us.ihmc.euclid.orientation.interfaces.Orientation3DReadOnly;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FrameVector3DBasics;
import us.ihmc.euclid.transform.interfaces.RigidBodyTransformBasics;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;
import us.ihmc.simulationconstructionset.util.CommonJoint;
import us.ihmc.yoVariables.euclid.YoVector3D;

public interface BallAndSocketSCSJoint extends CommonJoint
{
   public abstract void setOrientation(Orientation3DReadOnly orientation);

   public abstract void setAngularVelocityInBody(Vector3DReadOnly angularVelocityInBody);

   public abstract void getAngularVelocity(FrameVector3DBasics angularVelocityToPack, ReferenceFrame bodyFrame);

   public abstract void getTransformToWorld(RigidBodyTransformBasics ret);

   public abstract  void setJointTorque(Vector3DReadOnly jointTorque);

   public abstract  YoVector3D getJointTorque();
}
