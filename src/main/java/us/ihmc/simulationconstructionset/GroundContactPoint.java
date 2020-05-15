package us.ihmc.simulationconstructionset;

import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.tuple3D.interfaces.Point3DBasics;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Tuple3DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DBasics;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoFramePoint3D;
import us.ihmc.yoVariables.variable.YoFrameVector3D;
import us.ihmc.yoVariables.variable.YoInteger;

public class GroundContactPoint extends ExternalForcePoint
{
   private static final long serialVersionUID = 2334921180229856021L;

   private final YoFramePoint3D touchdownLocation;

   private final YoDouble fs; // Foot Switch TODO: YoBoolean or YoEnum
   private final YoFrameVector3D surfaceNormal;

   private final YoBoolean slip; // Whether or not it is slipping.
   private final YoInteger collisionCount;

   public GroundContactPoint(String name, Robot robot)
   {
      this(name, null, robot.getRobotsYoVariableRegistry());
   }

   public GroundContactPoint(String name, YoVariableRegistry registry)
   {
      this(name, null, registry);
   }

   public GroundContactPoint(String name, Tuple3DReadOnly offset, Robot robot)
   {
      this(name, offset, robot.getRobotsYoVariableRegistry());
   }

   public GroundContactPoint(String name, Tuple3DReadOnly offset, YoVariableRegistry registry)
   {
      super(name, offset, registry);

      touchdownLocation = new YoFramePoint3D(name + "_td", "", ReferenceFrame.getWorldFrame(), registry);

      fs = new YoDouble(name + "_fs", "GroundContactPoint foot switch", registry);

      slip = new YoBoolean(name + "_slip", "GroundContactPoint slipping", registry);
      collisionCount = new YoInteger(name + "_coll", "GroundContactPoint colliding", registry);

      surfaceNormal = new YoFrameVector3D(name + "_n", "", ReferenceFrame.getWorldFrame(), registry);
   }

   public boolean isSlipping()
   {
      return slip.getBooleanValue();
   }

   public boolean isInContact()
   {
      return (fs.getDoubleValue() > 0.5);
   }

   public void disable()
   {
      fs.set(-1.0);
   }

   public boolean isDisabled()
   {
      return (fs.getDoubleValue() < -0.5);
   }

   public void setIsSlipping(boolean isSlipping)
   {
      slip.set(isSlipping);
   }

   public int getCollisionCount()
   {
      return collisionCount.getIntegerValue();
   }

   public void incrementCollisionCount()
   {
      collisionCount.increment();
   }

   public void setInContact()
   {
      fs.set(1.0);
   }

   public void setNotInContact()
   {
      fs.set(0.0);
   }

   public void setIsInContact(boolean isInContact)
   {
      if (isInContact)
         setInContact();
      else
         setNotInContact();
   }

   public void getTouchdownLocation(Point3DBasics touchdownLocationToPack)
   {
      touchdownLocationToPack.set(touchdownLocation);
   }

   public YoFramePoint3D getYoTouchdownLocation()
   {
      return touchdownLocation;
   }

   public YoDouble getYoFootSwitch()
   {
      return fs;
   }

   public void setTouchdownLocation(Point3DReadOnly touchdownLocation)
   {
      this.touchdownLocation.set(touchdownLocation);
   }

   public void setTouchdownToCurrentLocation()
   {
      touchdownLocation.set(getYoPosition());
   }

   public void getSurfaceNormal(Vector3DBasics vectorToPack)
   {
      vectorToPack.set(surfaceNormal);
   }

   public void setSurfaceNormal(Vector3DReadOnly surfaceNormal)
   {
      this.surfaceNormal.set(surfaceNormal);
   }

   public void setSurfaceNormal(double fx, double fy, double fz)
   {
      surfaceNormal.set(fx, fy, fz);
   }

   public YoFrameVector3D getYoSurfaceNormal()
   {
      return surfaceNormal;
   }
}
