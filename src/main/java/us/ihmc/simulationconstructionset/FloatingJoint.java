package us.ihmc.simulationconstructionset;

import net.jafama.FastMath;
import us.ihmc.euclid.matrix.interfaces.RotationMatrixReadOnly;
import us.ihmc.euclid.orientation.interfaces.Orientation3DReadOnly;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FrameVector3DBasics;
import us.ihmc.euclid.transform.interfaces.RigidBodyTransformReadOnly;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple3D.interfaces.Tuple3DBasics;
import us.ihmc.euclid.tuple3D.interfaces.Tuple3DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DBasics;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.euclid.tuple4D.interfaces.QuaternionBasics;
import us.ihmc.euclid.tuple4D.interfaces.QuaternionReadOnly;
import us.ihmc.simulationconstructionset.physics.engine.featherstone.FloatingJointPhysics;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoPoint3D;
import us.ihmc.yoVariables.variable.YoQuaternion;
import us.ihmc.yoVariables.variable.YoVector3D;

public class FloatingJoint extends Joint implements FloatingSCSJoint
{
   private static final long serialVersionUID = 6863566500545068060L;

   private YoBoolean isPinned;

   public final YoDouble q_x, q_y, q_z; // in world-fixed frame
   public final YoDouble qd_x;
   public final YoDouble qd_y;
   public final YoDouble qd_z; // in world-fixed frame
   public final YoDouble q_qs;
   public final YoDouble q_qx;
   public final YoDouble q_qy;
   public final YoDouble q_qz; // Unit quaternion (Euler parameters). q_qs is the 'scalar part', q_q{x,y,z} form the vector part
   public final YoDouble qd_wx;
   public final YoDouble qd_wy;
   public final YoDouble qd_wz; // angular velocity, expressed in body-fixed frame.
   public final YoDouble qdd_x, qdd_y, qdd_z; // in world-fixed frame
   public final YoDouble qdd_wx, qdd_wy, qdd_wz; // angular acceleration, expressed in body-fixed frame.

   public final YoPoint3D position;
   public final YoQuaternion quaternion;
   public final YoVector3D linearVelocity;
   public final YoVector3D angularVelocity;
   public final YoVector3D linearAcceleration;
   public final YoVector3D angularAcceleration;

   private final boolean createYawPitchRollYoVariable;
   public final YoDouble q_yaw, q_pitch, q_roll; // in world-fixed frame.

   public FloatingJoint(String jname, Tuple3DReadOnly offset, Robot rob)
   {
      this(jname, null, offset, rob, false);
   }

   public FloatingJoint(String jname, Tuple3DReadOnly offset, Robot rob, boolean createYawPitchRollYoVariable)
   {
      this(jname, null, offset, rob, createYawPitchRollYoVariable);
   }

   public FloatingJoint(String jname, String varName, Tuple3DReadOnly offset, Robot rob)
   {
      this(jname, varName, offset, rob, false);
   }

   public FloatingJoint(String jname, String varName, Tuple3DReadOnly offset, Robot rob, boolean createYawPitchRollYoVariable)
   {
      super(jname, offset, rob, 6);

      physics = new FloatingJointPhysics(this);

      YoVariableRegistry registry = rob.getRobotsYoVariableRegistry();

      this.createYawPitchRollYoVariable = createYawPitchRollYoVariable;

      if (varName == null)
      {
         varName = "";
      }
      else if (!varName.isEmpty())
      {
         varName += "_";
      }

      q_x = new YoDouble("q_" + varName + "x", "FloatingJoint x position", registry);
      q_y = new YoDouble("q_" + varName + "y", "FloatingJoint y position", registry);
      q_z = new YoDouble("q_" + varName + "z", "FloatingJoint z position", registry);
      qd_x = new YoDouble("qd_" + varName + "x", "FloatingJoint x velocity", registry);
      qd_y = new YoDouble("qd_" + varName + "y", "FloatingJoint y velocity", registry);
      qd_z = new YoDouble("qd_" + varName + "z", "FloatingJoint z velocity", registry);
      qdd_x = new YoDouble("qdd_" + varName + "x", "FloatingJoint x acceleration", registry);
      qdd_y = new YoDouble("qdd_" + varName + "y", "FloatingJoint yx acceleration", registry);
      qdd_z = new YoDouble("qdd_" + varName + "z", "FloatingJoint z acceleration", registry);
      q_qs = new YoDouble("q_" + varName + "qs", "FloatingJoint orientation quaternion qs", registry);
      q_qs.set(1.0);
      q_qx = new YoDouble("q_" + varName + "qx", "FloatingJoint orientation quaternion qx", registry);
      q_qy = new YoDouble("q_" + varName + "qy", "FloatingJoint orientation quaternion qy", registry);
      q_qz = new YoDouble("q_" + varName + "qz", "FloatingJoint orientation quaternion qz", registry);
      qd_wx = new YoDouble("qd_" + varName + "wx", "FloatingJoint rotational velocity about x", registry);
      qd_wy = new YoDouble("qd_" + varName + "wy", "FloatingJoint rotational velocity about y", registry);
      qd_wz = new YoDouble("qd_" + varName + "wz", "FloatingJoint rotational velocity about z", registry);
      qdd_wx = new YoDouble("qdd_" + varName + "wx", "FloatingJoint rotational acceleration about x", registry);
      qdd_wy = new YoDouble("qdd_" + varName + "wy", "FloatingJoint rotational acceleration about y", registry);
      qdd_wz = new YoDouble("qdd_" + varName + "wz", "FloatingJoint rotational acceleration about z", registry);

      position = new YoPoint3D(q_x, q_y, q_z);
      quaternion = new YoQuaternion(q_qx, q_qy, q_qz, q_qs);
      linearVelocity = new YoVector3D(qd_x, qd_y, qd_z);
      angularVelocity = new YoVector3D(qd_wx, qd_wy, qd_wz);
      linearAcceleration = new YoVector3D(qdd_x, qdd_y, qdd_z);
      angularAcceleration = new YoVector3D(qdd_wx, qdd_wy, qdd_wz);

      if (createYawPitchRollYoVariable)
      {
         q_yaw = new YoDouble("q_" + varName + "yaw", "FloatingJoint yaw orientation", registry);
         q_pitch = new YoDouble("q_" + varName + "pitch", "FloatingJoint pitch orientation", registry);
         q_roll = new YoDouble("q_" + varName + "roll", "FloatingJoint roll orientation", registry);
      }
      else
      {
         q_yaw = null;
         q_pitch = null;
         q_roll = null;
      }

      isPinned = new YoBoolean(jname + "IsPinned", "Whether this FloatingJoint is pinned or not", registry);

      jointTransform3D.set(quaternion, position);
      physics.u_i = null;
   }

   public void setPositionAndVelocity(double x, double y, double z, double dx, double dy, double dz)
   {
      position.set(x, y, z);
      linearVelocity.set(dx, dy, dz);
   }

   public void setPositionAndVelocity(Tuple3DReadOnly position, Tuple3DReadOnly velocity)
   {
      this.position.set(position);
      linearVelocity.set(velocity);
   }

   public void setPosition(Tuple3DReadOnly position)
   {
      this.position.set(position);
   }

   public void setPosition(double x, double y, double z)
   {
      position.set(x, y, z);
   }

   @Override
   public void setVelocity(Tuple3DReadOnly velocity)
   {
      linearVelocity.set(velocity);
   }

   public void setVelocity(double xd, double yd, double zd)
   {
      linearVelocity.set(xd, yd, zd);
   }

   public void setAcceleration(Tuple3DReadOnly acceleration)
   {
      linearAcceleration.set(acceleration);
   }

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
   public void setRotationAndTranslation(RigidBodyTransformReadOnly transform)
   {
      setOrientation(transform.getRotation());
      setPosition(transform.getTranslation());
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

   public void getPosition(YoDouble x, YoDouble y, YoDouble z)
   {
      x.set(q_x.getDoubleValue());
      y.set(q_y.getDoubleValue());
      z.set(q_z.getDoubleValue());
   }

   public void getVelocity(YoDouble xDot, YoDouble yDot, YoDouble zDot)
   {
      xDot.set(qd_x.getDoubleValue());
      yDot.set(qd_y.getDoubleValue());
      zDot.set(qd_z.getDoubleValue());
   }

   @Override
   public void getVelocity(FrameVector3DBasics linearVelocityToPack)
   {
      linearVelocityToPack.setIncludingFrame(ReferenceFrame.getWorldFrame(), linearVelocity);
   }

   @Override
   public void getAngularVelocity(FrameVector3DBasics angularVelocityToPack, ReferenceFrame bodyFrame)
   {
      angularVelocityToPack.setIncludingFrame(bodyFrame, angularVelocity);
   }

   public void getPositionAndVelocity(YoDouble x, YoDouble y, YoDouble z, YoDouble xDot, YoDouble yDot, YoDouble zDot)
   {
      getPosition(x, y, z);
      getVelocity(xDot, yDot, zDot);
   }

   public void getPosition(Tuple3DBasics position)
   {
      position.set(this.position);
   }

   public void getVelocity(Tuple3DBasics velocity)
   {
      velocity.set(linearVelocity);
   }

   public void getPositionAndVelocity(Tuple3DBasics position, Tuple3DBasics velocity)
   {
      getPosition(position);
      getVelocity(velocity);
   }

   public YoDouble getQx()
   {
      return q_x;
   }

   public YoDouble getQy()
   {
      return q_y;
   }

   public YoDouble getQz()
   {
      return q_z;
   }

   public YoPoint3D getPosition()
   {
      return position;
   }

   public YoDouble getQdx()
   {
      return qd_x;
   }

   public YoDouble getQdy()
   {
      return qd_y;
   }

   public YoDouble getQdz()
   {
      return qd_z;
   }

   public YoVector3D getLinearVelocity()
   {
      return linearVelocity;
   }

   public YoDouble getQddx()
   {
      return qdd_x;
   }

   public YoDouble getQddy()
   {
      return qdd_y;
   }

   public YoDouble getQddz()
   {
      return qdd_z;
   }

   public YoVector3D getLinearAcceleration()
   {
      return linearAcceleration;
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

   public void getLinearAccelerationInWorld(Vector3DBasics accelerationInWorldToPack)
   {
      accelerationInWorldToPack.set(linearAcceleration);
   }

   public void getLinearAcceleration(FrameVector3DBasics linearAccelerationToPack)
   {
      linearAccelerationToPack.setIncludingFrame(ReferenceFrame.getWorldFrame(), linearAcceleration);
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
    * Indicates whether this floating joint is fixed or not. This field differs from
    * {@link #isDynamic()} as when it is pinned, the subtree attached to this joint is still simulated.
    */
   public boolean isPinned()
   {
      return isPinned.getValue();
   }

   /**
    * Indicates whether this floating joint is fixed or not. This field differs from
    * {@link #isDynamic()} as when it is pinned, the subtree attached to this joint is still simulated.
    */
   public void setPinned(boolean isPinned)
   {
      this.isPinned.set(isPinned);
   }

   @Override
   public void update()
   {
      jointTransform3D.set(quaternion, position);

      if (createYawPitchRollYoVariable)
      {
         getYawPitchRoll(q_yaw, q_pitch, q_roll);
      }
   }
}
