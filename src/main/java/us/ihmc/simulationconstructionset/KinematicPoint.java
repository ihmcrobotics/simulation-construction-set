package us.ihmc.simulationconstructionset;

import us.ihmc.euclid.matrix.interfaces.RotationMatrixReadOnly;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.transform.interfaces.RigidBodyTransformReadOnly;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Tuple3DBasics;
import us.ihmc.euclid.tuple3D.interfaces.Tuple3DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DBasics;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;
import us.ihmc.euclid.tuple4D.Vector4D;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoFramePoint3D;
import us.ihmc.yoVariables.variable.YoFrameVector3D;

/**
 * Title: Simulation Construction Set
 * <p>
 * Description: Package for Simulating Dynamic Robots and Mechanisms
 * <p>
 * 
 * @author Jerry Pratt
 * @version Beta 1.0
 */
public class KinematicPoint implements java.io.Serializable
{
   private static final long serialVersionUID = 3047881738704107434L;

   private final String name;

   // The position and velocity are in world frame. AngularVelocity is in body frame of the joint.
   private final YoFramePoint3D positionInWorld;
   private final YoFrameVector3D velocityInWorld;
   private final YoFrameVector3D angularVelocityInWorld;

   private final YoFrameVector3D offsetYoFrameVector;

   protected Joint parentJoint;

   private KinematicPointUpdater kinematicPointUpdater;

   private final YoVariableRegistry registry;

   protected final Vector3D tempVectorForOffsetFromCOM = new Vector3D(), tempVectorForWXr = new Vector3D(), tempVectorForVelocity = new Vector3D();

   private RigidBodyTransform tempTransformFromWorldToJoint = new RigidBodyTransform();
   private Vector4D offsetPlus = new Vector4D();

   private final Point3D tempPoint = new Point3D();

   public KinematicPoint(String name, Robot robot)
   {
      this(name, null, robot.getRobotsYoVariableRegistry());
   }

   public KinematicPoint(String name, YoVariableRegistry registry)
   {
      this(name, null, registry);
   }

   public KinematicPoint(String name, Tuple3DReadOnly offset, Robot robot)
   {
      this(name, offset, robot.getRobotsYoVariableRegistry());
   }

   public KinematicPoint(String name, Tuple3DReadOnly offset, YoVariableRegistry registry)
   {
      this.name = name;
      this.registry = registry;

      positionInWorld = new YoFramePoint3D(name + "_", "", ReferenceFrame.getWorldFrame(), registry);
      velocityInWorld = new YoFrameVector3D(name + "_d", "", ReferenceFrame.getWorldFrame(), registry);
      angularVelocityInWorld = new YoFrameVector3D(name + "_w", "", ReferenceFrame.getWorldFrame(), registry);

      offsetYoFrameVector = new YoFrameVector3D(name + "off", "", ReferenceFrame.getWorldFrame(), registry);
      if (offset != null)
         offsetYoFrameVector.set(offset);
   }

   public void reset()
   {
      parentJoint = null;

      offsetYoFrameVector.set(0, 0, 0);

      tempVectorForOffsetFromCOM.set(0, 0, 0);
      tempVectorForWXr.set(0, 0, 0);
      tempVectorForVelocity.set(0, 0, 0);

      tempTransformFromWorldToJoint.setIdentity();
      offsetPlus.set(0, 0, 0, 0);

      positionInWorld.set(0, 0, 0);
      velocityInWorld.set(0, 0, 0);
      angularVelocityInWorld.set(0, 0, 0);
   }

   public KinematicPointUpdater getKinematicPointUpdater()
   {
      return kinematicPointUpdater;
   }

   public void setKinematicPointUpdater(KinematicPointUpdater updater)
   {
      kinematicPointUpdater = updater;
   }

   public void setParentJoint(Joint parent)
   {
      parentJoint = parent;
   }

   public Joint getParentJoint()
   {
      return parentJoint;
   }

   @Override
   public String toString()
   {
      return ("name: " + name + " x: " + positionInWorld.getX() + ", y: " + positionInWorld.getY() + ", z: " + positionInWorld.getZ());
   }

   public void setOffsetJoint(double x, double y, double z)
   {
      offsetYoFrameVector.set(x, y, z);
   }

   public void setOffsetJoint(Tuple3DReadOnly newOffset)
   {
      offsetYoFrameVector.set(newOffset);
   }

   public void setOffsetWorld(Tuple3DReadOnly offsetInWorld)
   {
      setOffsetWorld(offsetInWorld.getX(), offsetInWorld.getY(), offsetInWorld.getZ());
   }

   public void setOffsetWorld(double x, double y, double z)
   {
      tempTransformFromWorldToJoint.set(parentJoint.transformToNext);
      tempTransformFromWorldToJoint.invert();
      offsetPlus.set(x, y, z, 1.0);
      tempTransformFromWorldToJoint.transform(offsetPlus);

      setOffsetJoint(offsetPlus.getX(), offsetPlus.getY(), offsetPlus.getZ());

      //TODO: Make sure all methods update the various variables so that a set followed by a get is consistent...
      positionInWorld.set(x, y, z);
   }

   public void updatePointVelocity(RotationMatrixReadOnly R0_i, Vector3DReadOnly comOffset, Vector3DReadOnly v_i, Vector3DReadOnly w_i)
   {
      getOffset(tempVectorForOffsetFromCOM);
      tempVectorForOffsetFromCOM.sub(comOffset);

      tempVectorForWXr.cross(w_i, tempVectorForOffsetFromCOM);
      tempVectorForVelocity.add(v_i, tempVectorForWXr);

      R0_i.transform(tempVectorForVelocity);
      velocityInWorld.set(tempVectorForVelocity);

      tempVectorForVelocity.set(w_i);
      R0_i.transform(tempVectorForVelocity);
      angularVelocityInWorld.set(tempVectorForVelocity);
   }

   protected void updatePointPosition(RigidBodyTransformReadOnly t1)
   {
      if (kinematicPointUpdater != null)
      {
         // System.out.print(".");
         kinematicPointUpdater.updateKinematicPoint(this);
      }

      getOffset(tempPoint);
      t1.transform(tempPoint);

      positionInWorld.set(tempPoint);
   }

   public String getName()
   {
      return name;
   }

   public Vector3DReadOnly getOffset()
   {
      return offsetYoFrameVector;
   }

   public void getOffset(Tuple3DBasics offsetToPack)
   {
      offsetToPack.set(offsetYoFrameVector);
   }

   public Vector3D getOffsetCopy()
   {
      return new Vector3D(getOffset());
   }

   public double getX()
   {
      return positionInWorld.getX();
   }

   public double getY()
   {
      return positionInWorld.getY();
   }

   public double getZ()
   {
      return positionInWorld.getZ();
   }

   public double getXVelocity()
   {
      return velocityInWorld.getX();
   }

   public double getYVelocity()
   {
      return velocityInWorld.getY();
   }

   public double getZVelocity()
   {
      return velocityInWorld.getZ();
   }

   public void getPosition(Tuple3DBasics positionToPack)
   {
      positionToPack.set(positionInWorld);
   }

   /**
    * @deprecated Use {@link #getPositionCopy()} instead
    */
   public Point3D getPositionPoint()
   {
      return getPositionCopy();
   }

   public Point3D getPositionCopy()
   {
      return new Point3D(positionInWorld);
   }

   public void getVelocity(Vector3DBasics velocityToPack)
   {
      velocityToPack.set(velocityInWorld);
   }

   /**
    * @deprecated Use {@link #getVelocityCopy()} instead
    */
   public Vector3D getVelocityVector()
   {
      return getVelocityCopy();
   }

   public Vector3D getVelocityCopy()
   {
      return new Vector3D(velocityInWorld);
   }

   public void setVelocity(Vector3DReadOnly velocity)
   {
      velocityInWorld.set(velocity);
   }

   public Vector3DReadOnly getAngularVelocity()
   {
      return angularVelocityInWorld;
   }

   public void getAngularVelocity(Vector3DBasics angularVelocityInWorldToPack)
   {
      angularVelocityInWorldToPack.set(angularVelocityInWorld);
   }

   public void setAngularVelocity(Vector3DReadOnly angularVelocityInWorld)
   {
      this.angularVelocityInWorld.set(angularVelocityInWorld);
   }

   public void setPosition(Point3DReadOnly position)
   {
      positionInWorld.set(position);
   }

   public YoFramePoint3D getYoPosition()
   {
      return positionInWorld;
   }

   public YoFrameVector3D getYoVelocity()
   {
      return velocityInWorld;
   }

   public YoFrameVector3D getYoAngularVelocity()
   {
      return angularVelocityInWorld;
   }

   public YoVariableRegistry getYoVariableRegistry()
   {
      return registry;
   }
}
