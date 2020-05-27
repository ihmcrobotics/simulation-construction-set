package us.ihmc.simulationconstructionset;

import us.ihmc.euclid.transform.interfaces.RigidBodyTransformBasics;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple3D.interfaces.Tuple3DBasics;
import us.ihmc.euclid.tuple3D.interfaces.Tuple3DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;
import us.ihmc.yoVariables.variable.YoFrameVector3D;

public class JointWrenchSensor
{
   private final String name;
   private final YoFrameVector3D jointWrenchForce, jointWrenchTorque;
   private final Vector3D offsetFromJoint = new Vector3D();

   // FIXME offsetFromJoint is probably the offsetToJoint.
   public JointWrenchSensor(String name, Tuple3DReadOnly offsetFromJoint, Robot robot)
   {
      this.name = name;

      jointWrenchForce = new YoFrameVector3D(name + "_f", null, robot.getRobotsYoVariableRegistry());
      jointWrenchTorque = new YoFrameVector3D(name + "_t", null, robot.getRobotsYoVariableRegistry());

      this.offsetFromJoint.set(offsetFromJoint);
   }

   public void getOffsetFromJoint(Tuple3DBasics offsetFromJointToPack)
   {
      offsetFromJointToPack.set(offsetFromJoint);
   }

   public void getJointForce(Tuple3DBasics forceToPack)
   {
      forceToPack.set(jointWrenchForce);
   }

   public void getJointTorque(Tuple3DBasics torqueToPack)
   {
      torqueToPack.set(jointWrenchTorque);
   }

   private final Vector3D tempVector = new Vector3D();

   public void setWrench(SpatialVector wrenchToSet)
   {
      wrenchToSet.getTop(tempVector);
      jointWrenchForce.set(tempVector);

      wrenchToSet.getBottom(tempVector);
      jointWrenchTorque.set(tempVector);
   }

   public Vector3DReadOnly getOffsetFromJoint()
   {
      return offsetFromJoint;
   }

   public void getTransformToParentJoint(RigidBodyTransformBasics transformToPack)
   {
      transformToPack.setTranslationAndIdentityRotation(offsetFromJoint);
   }

   public String getName()
   {
      return name;
   }
}
