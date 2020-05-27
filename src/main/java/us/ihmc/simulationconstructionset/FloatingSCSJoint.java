package us.ihmc.simulationconstructionset;

import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FrameVector3DBasics;
import us.ihmc.euclid.transform.interfaces.RigidBodyTransformBasics;
import us.ihmc.euclid.transform.interfaces.RigidBodyTransformReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Tuple3DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;
import us.ihmc.simulationconstructionset.util.CommonJoint;

public interface FloatingSCSJoint extends CommonJoint
{
   public void setRotationAndTranslation(RigidBodyTransformReadOnly transform);

   public void setVelocity(Tuple3DReadOnly velocity);

   public void setAngularVelocityInBody(Vector3DReadOnly angularVelocityInBody);

   public void getVelocity(FrameVector3DBasics linearVelocityToPack);

   public void getAngularVelocity(FrameVector3DBasics angularVelocityToPack, ReferenceFrame bodyFrame);

   public void getTransformToWorld(RigidBodyTransformBasics ret);
}
