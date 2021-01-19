package us.ihmc.simulationconstructionset;

import net.jafama.FastMath;
import us.ihmc.euclid.matrix.interfaces.RotationMatrixReadOnly;
import us.ihmc.euclid.orientation.interfaces.Orientation3DReadOnly;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FrameVector3DBasics;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple3D.interfaces.Tuple3DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DBasics;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.euclid.tuple4D.interfaces.QuaternionBasics;
import us.ihmc.euclid.tuple4D.interfaces.QuaternionReadOnly;
import us.ihmc.simulationconstructionset.physics.engine.featherstone.BallAndSocketJointPhysics;
import us.ihmc.yoVariables.euclid.YoQuaternion;
import us.ihmc.yoVariables.euclid.YoVector3D;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoDouble;

public class BallAndSocketJoint extends Joint implements BallAndSocketSCSJoint
{
   private static final long serialVersionUID = 6863566500545068060L;

   private YoBoolean isPinned;

   public final YoDouble q_qs;
   public final YoDouble q_qx;
   public final YoDouble q_qy;
   public final YoDouble q_qz; // Unit quaternion (Euler parameters). q_qs is the 'scalar part', q_q{x,y,z} form the vector part
   public final YoDouble qd_wx;
   public final YoDouble qd_wy;
   public final YoDouble qd_wz; // angular velocity, expressed in body-fixed frame.
   public final YoDouble qdd_wx, qdd_wy, qdd_wz; // angular acceleration, expressed in body-fixed frame.

   public final YoDouble tau_wx, tau_wy, tau_wz;

   public final YoQuaternion quaternion;
   public final YoVector3D angularVelocity;
   public final YoVector3D angularAcceleration;
   public final YoVector3D torqueVector;

   private final boolean createYawPitchRollYoVariable;
   public final YoDouble q_yaw, q_pitch, q_roll; // in world-fixed frame.

   public BallAndSocketJoint(String name, Tuple3DReadOnly offset, Robot rob)
   {
      this(name, offset, rob, false);
   }

   public BallAndSocketJoint(String name, Tuple3DReadOnly offset, Robot rob, boolean createYawPitchRollYoVariable)
   {
      super(name, offset, rob, 6);

      physics = new BallAndSocketJointPhysics(this);

      YoRegistry registry = rob.getRobotsYoRegistry();

      this.createYawPitchRollYoVariable = createYawPitchRollYoVariable;

      if (name == null)
      {
         name = "";
      }
      else if (!name.isEmpty())
      {
         name += "_";
      }

      q_qs = new YoDouble("q_" + name + "qs", "BallAndSocketJoint orientation quaternion qs", registry);
      q_qs.set(1.0);
      q_qx = new YoDouble("q_" + name + "qx", "BallAndSocketJoint orientation quaternion qx", registry);
      q_qy = new YoDouble("q_" + name + "qy", "BallAndSocketJoint orientation quaternion qy", registry);
      q_qz = new YoDouble("q_" + name + "qz", "BallAndSocketJoint orientation quaternion qz", registry);
      qd_wx = new YoDouble("qd_" + name + "wx", "BallAndSocketJoint rotational velocity about x", registry);
      qd_wy = new YoDouble("qd_" + name + "wy", "BallAndSocketJoint rotational velocity about y", registry);
      qd_wz = new YoDouble("qd_" + name + "wz", "BallAndSocketJoint rotational velocity about z", registry);
      qdd_wx = new YoDouble("qdd_" + name + "wx", "BallAndSocketJoint rotational acceleration about x", registry);
      qdd_wy = new YoDouble("qdd_" + name + "wy", "BallAndSocketJoint rotational acceleration about y", registry);
      qdd_wz = new YoDouble("qdd_" + name + "wz", "BallAndSocketJoint rotational acceleration about z", registry);
      
      tau_wx = new YoDouble("tau_" + name + "wx", "BallAndSocketJoint torque about x", registry);
      tau_wy = new YoDouble("tau_" + name + "wy", "BallAndSocketJoint torque about y", registry);
      tau_wz = new YoDouble("tau_" + name + "wz", "BallAndSocketJoint torque about z", registry);

      quaternion = new YoQuaternion(q_qx, q_qy, q_qz, q_qs);
      angularVelocity = new YoVector3D(qd_wx, qd_wy, qd_wz);
      angularAcceleration = new YoVector3D(qdd_wx, qdd_wy, qdd_wz);
      torqueVector = new YoVector3D(tau_wx, tau_wy, tau_wz);

      if (createYawPitchRollYoVariable)
      {
         q_yaw = new YoDouble("q_" + name + "yaw", "BallAndSocketJoint yaw orientation", registry);
         q_pitch = new YoDouble("q_" + name + "pitch", "BallAndSocketJoint pitch orientation", registry);
         q_roll = new YoDouble("q_" + name + "roll", "BallAndSocketJoint roll orientation", registry);
      }
      else
      {
         q_yaw = null;
         q_pitch = null;
         q_roll = null;
      }

      isPinned = new YoBoolean(name + "IsPinned", "Whether this BallAndSocketJoint is pinned or not", registry);

      jointTransform3D.getRotation().set(quaternion);
      
      physics.u_i = null;
   }

   @Override
   public void setOrientation(Orientation3DReadOnly orientation)
   {
      quaternion.set(orientation);
   }

   public void setYawPitchRoll(double yaw, double pitch, double roll)
   {
      quaternion.setYawPitchRoll(yaw, pitch, roll);
   }

   public void setYawPitchRoll(double yaw, double pitch, double roll, double wz, double wy, double wx)
   {
      setYawPitchRoll(yaw, pitch, roll);
      angularVelocity.set(wx, wy, wz);
   }

   /**
    * @deprecated Use {@link #setOrientation(Orientation3DReadOnly)} instead.
    */
   public void setRotation(RotationMatrixReadOnly rotation)
   {
      setOrientation((Orientation3DReadOnly) rotation);
   }

   /**
    * @deprecated Use {@link #setOrientation(Orientation3DReadOnly)} instead.
    */
   public void setQuaternion(QuaternionReadOnly q)
   {
      setOrientation((Orientation3DReadOnly) q);
   }

   @Override
   public void setAngularVelocityInBody(Vector3DReadOnly angularVelocityInBody)
   {
      angularVelocity.set(angularVelocityInBody);
   }

   public void setAngularAccelerationInBody(Vector3DReadOnly angularAccelerationInBody)
   {
      angularAcceleration.set(angularAccelerationInBody);
   }


   @Override
   public void getAngularVelocity(FrameVector3DBasics angularVelocityToPack, ReferenceFrame bodyFrame)
   {
      angularVelocityToPack.setIncludingFrame(bodyFrame, angularVelocity);
   }

   public YoDouble getQuaternionQs()
   {
      return q_qs;
   }

   public YoDouble getQuaternionQx()
   {
      return q_qx;
   }

   public YoDouble getQuaternionQy()
   {
      return q_qy;
   }

   public YoDouble getQuaternionQz()
   {
      return q_qz;
   }

   public YoQuaternion getOrientation()
   {
      return quaternion;
   }

   public Quaternion getQuaternion()
   {
      return new Quaternion(quaternion);
   }

   public void getQuaternion(QuaternionBasics quaternionToPack)
   {
      quaternionToPack.set(quaternion);
   }

   public YoDouble getAngularVelocityX()
   {
      return qd_wx;
   }

   public YoDouble getAngularVelocityY()
   {
      return qd_wy;
   }

   public YoDouble getAngularVelocityZ()
   {
      return qd_wz;
   }

   public YoVector3D getAngularVelocity()
   {
      return angularVelocity;
   }

   public Vector3D getAngularVelocityInBody()
   {
      return new Vector3D(angularVelocity);
   }

   public void getAngularVelocityInBody(Vector3DBasics vectorToPack)
   {
      vectorToPack.set(angularVelocity);
   }

   public YoDouble getAngularAccelerationX()
   {
      return qdd_wx;
   }

   public YoDouble getAngularAccelerationY()
   {
      return qdd_wy;
   }

   public YoDouble getAngularAccelerationZ()
   {
      return qdd_wz;
   }

   public YoVector3D getAngularAcceleration()
   {
      return angularAcceleration;
   }

   public Vector3D getAngularAccelerationInBody()
   {
      return new Vector3D(angularAcceleration);
   }

   public void getAngularAccelerationInBody(Vector3DBasics angularAccelerationInBodyToPack)
   {
      angularAccelerationInBodyToPack.set(angularAcceleration);
   }

   public void getAngularAcceleration(FrameVector3DBasics angularAccelerationToPack, ReferenceFrame bodyFrame)
   {
      angularAccelerationToPack.setIncludingFrame(bodyFrame, angularAcceleration);
   }

   public void getYawPitchRoll(YoDouble yaw, YoDouble pitch, YoDouble roll)
   {

      double pitchArgument = -2.0 * q_qx.getDoubleValue() * q_qz.getDoubleValue() + 2.0 * q_qs.getDoubleValue() * q_qy.getDoubleValue();

      pitch.set(FastMath.asin(pitchArgument));

      if (Math.abs(pitch.getDoubleValue()) < 0.49 * Math.PI)
      {
         yaw.set(FastMath.atan2(2.0 * q_qx.getDoubleValue() * q_qy.getDoubleValue() + 2.0 * q_qz.getDoubleValue() * q_qs.getDoubleValue(),
                                1.0 - 2.0 * q_qy.getDoubleValue() * q_qy.getDoubleValue() - 2.0 * q_qz.getDoubleValue() * q_qz.getDoubleValue())); // Math.asin(q_qs.val * q_qz.val * 2.0);
         roll.set(FastMath.atan2(2.0 * q_qy.getDoubleValue() * q_qz.getDoubleValue() + 2.0 * q_qx.getDoubleValue() * q_qs.getDoubleValue(),
                                 1.0 - 2.0 * q_qx.getDoubleValue() * q_qx.getDoubleValue() - 2.0 * q_qy.getDoubleValue() * q_qy.getDoubleValue())); // Math.asin(q_qs.val * q_qx.val * 2.0);
      }
      else
      {
         yaw.set(2.0 * FastMath.atan2(q_qz.getDoubleValue(), q_qs.getDoubleValue()));
         roll.set(0.0);
      }
   }

   public double[] getYawPitchRoll()
   {
      double[] yawPitchRollToReturn = new double[3];

      double pitchArgument = -2.0 * q_qx.getDoubleValue() * q_qz.getDoubleValue() + 2.0 * q_qs.getDoubleValue() * q_qy.getDoubleValue();

      double pitch = 0.0, roll = 0.0, yaw = 0.0;

      pitch = FastMath.asin(pitchArgument);

      if (Math.abs(pitch) < 0.49 * Math.PI)
      {
         yaw = FastMath.atan2(2.0 * q_qx.getDoubleValue() * q_qy.getDoubleValue() + 2.0 * q_qz.getDoubleValue() * q_qs.getDoubleValue(),
                              1.0 - 2.0 * q_qy.getDoubleValue() * q_qy.getDoubleValue() - 2.0 * q_qz.getDoubleValue() * q_qz.getDoubleValue()); // Math.asin(q_qs.val * q_qz.val * 2.0);
         roll = FastMath.atan2(2.0 * q_qy.getDoubleValue() * q_qz.getDoubleValue() + 2.0 * q_qx.getDoubleValue() * q_qs.getDoubleValue(),
                               1.0 - 2.0 * q_qx.getDoubleValue() * q_qx.getDoubleValue() - 2.0 * q_qy.getDoubleValue() * q_qy.getDoubleValue()); // Math.asin(q_qs.val * q_qx.val * 2.0);
      }
      else
      {
         yaw = 2.0 * FastMath.atan2(q_qz.getDoubleValue(), q_qs.getDoubleValue());
         roll = 0.0;
      }

      yawPitchRollToReturn[0] = yaw;
      yawPitchRollToReturn[1] = pitch;
      yawPitchRollToReturn[2] = roll;

      return yawPitchRollToReturn;
   }

   /**
    * Indicates whether this BallAndSocketJoint is fixed or not. This field differs from
    * {@link #isDynamic()} as when it is pinned, the subtree attached to this joint is still simulated.
    */
   public boolean isPinned()
   {
      return isPinned.getValue();
   }

   /**
    * Indicates whether this BallAndSocketJoint is fixed or not. This field differs from
    * {@link #isDynamic()} as when it is pinned, the subtree attached to this joint is still simulated.
    */
   public void setPinned(boolean isPinned)
   {
      this.isPinned.set(isPinned);
   }

   @Override
   public void update()
   {
      jointTransform3D.getRotation().set(quaternion);

      if (createYawPitchRollYoVariable)
      {
         getYawPitchRoll(q_yaw, q_pitch, q_roll);
      }
   }

   @Override
   public void setJointTorque(Vector3DReadOnly jointTorque)
   {
      this.torqueVector.set(jointTorque);
   }

   @Override
   public YoVector3D getJointTorque()
   {
      return this.torqueVector;
   }
}
