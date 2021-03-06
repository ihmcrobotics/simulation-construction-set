package us.ihmc.simulationconstructionset;

import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.transform.interfaces.RigidBodyTransformBasics;
import us.ihmc.euclid.transform.interfaces.RigidBodyTransformReadOnly;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Tuple3DReadOnly;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.jMonkeyEngineToolkit.camera.CameraConfiguration;
import us.ihmc.jMonkeyEngineToolkit.camera.CameraMountInterface;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoDouble;

public class CameraMount implements CameraMountInterface
{
   private final String name;

   private final RigidBodyTransform offsetTransform;
   private final RigidBodyTransform transformToMount = new RigidBodyTransform();
   private final RigidBodyTransform transformToCamera = new RigidBodyTransform();

   private Joint parentJoint;

   private final Robot rob;

   private YoDouble pan, tilt, roll;

   private double fieldOfView, clipDistanceNear, clipDistanceFar;
   private int imageWidth, imageHeight;

   public CameraMount(String name, Tuple3DReadOnly offsetVector, Robot rob)
   {
      this(name, offsetVector, CameraConfiguration.DEFAULT_FIELD_OF_VIEW, CameraConfiguration.DEFAULT_CLIP_DISTANCE_NEAR,
           CameraConfiguration.DEFAULT_CLIP_DISTANCE_FAR, rob);
   }

   public CameraMount(String name, RigidBodyTransformReadOnly camPose, Robot rob)
   {
      this(name, camPose, CameraConfiguration.DEFAULT_FIELD_OF_VIEW, CameraConfiguration.DEFAULT_CLIP_DISTANCE_NEAR,
           CameraConfiguration.DEFAULT_CLIP_DISTANCE_FAR, rob);
   }

   public CameraMount(String name, Tuple3DReadOnly offsetVector, double fieldOfView, double clipDistanceNear, double clipDistanceFar, Robot rob)
   {
      this(name, new RigidBodyTransform(new Quaternion(), offsetVector), fieldOfView, clipDistanceNear, clipDistanceFar, rob);
   }

   public CameraMount(String name, RigidBodyTransformReadOnly offset, double fieldOfView, double clipDistanceNear, double clipDistanceFar, Robot rob)
   {
      this.name = name;
      this.rob = rob;

      offsetTransform = new RigidBodyTransform(offset);

      this.fieldOfView = fieldOfView;
      this.clipDistanceNear = clipDistanceNear;
      this.clipDistanceFar = clipDistanceFar;
   }

   @Override
   public String getName()
   {
      return name;
   }

   protected void setParentJoint(Joint parent)
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
      return ("name: " + name);
   }

   private boolean enablePanTiltRoll = false;
   private RigidBodyTransform panTiltRollTransform3D, temp1;

   public void enablePanTiltRoll()
   {
      YoRegistry registry = new YoRegistry("CameraMount");

      pan = new YoDouble("pan_" + name, registry);
      tilt = new YoDouble("tilt_" + name, registry);
      roll = new YoDouble("roll_" + name, registry);

      panTiltRollTransform3D = new RigidBodyTransform();
      temp1 = new RigidBodyTransform();

      enablePanTiltRoll = true;

      rob.addYoRegistry(registry);
   }

   private Point3D tempPoint3d;

   public void lookAt(Point3DReadOnly center)
   {
      lookAt(center.getX(), center.getY(), center.getZ());
   }

   public RigidBodyTransform lookAtTransform3D;

   public void lookAt(double x, double y, double z)
   {
      if (tempPoint3d == null)
         tempPoint3d = new Point3D();
      if (lookAtTransform3D == null)
         lookAtTransform3D = new RigidBodyTransform();

      // Make camera look at the point "center" by adjusting pan and tilt. Roll doesn't change.
      // This is fairly involved since the camera can be all willy nilly.
      tempPoint3d.set(x, y, z);

      lookAtTransform3D.set(transformToMount);
      lookAtTransform3D.invert();

      lookAtTransform3D.transform(tempPoint3d); // Put center from world coordinates to mount coordinates.

      // Compute pan and tilt to get there.
      pan.set(Math.atan2(tempPoint3d.getY(), tempPoint3d.getX()));
      tilt.set(Math.atan2(-tempPoint3d.getZ(), Math.sqrt(tempPoint3d.getX() * tempPoint3d.getX() + tempPoint3d.getY() * tempPoint3d.getY())));
   }

   protected void updateTransform(RigidBodyTransformReadOnly t1)
   {
      transformToMount.set(t1); // transformToMount. = t1 * offsetTransform;
      transformToMount.multiply(offsetTransform); // transformToMount. = t1 * offsetTransform;

      if (enablePanTiltRoll)
      {
         // Note that pan, tilt, roll are in z, y, x ccordinates.
         // transformToCamera = transformToMount * panTiltRollTransform3D

         panTiltRollTransform3D.setRotationYawAndZeroTranslation(pan.getDoubleValue());
         temp1.setRotationPitchAndZeroTranslation(tilt.getDoubleValue());
         panTiltRollTransform3D.multiply(temp1);
         temp1.setRotationRollAndZeroTranslation(roll.getDoubleValue());
         panTiltRollTransform3D.multiply(temp1);

         transformToCamera.set(transformToMount);
         transformToCamera.multiply(panTiltRollTransform3D);
      }

      else
      {
         transformToCamera.set(transformToMount);
      }

   }

   public void getTransformToMount(RigidBodyTransformBasics transformToPack)
   {
      transformToPack.set(transformToMount);
   }

   @Override
   public void getTransformToCamera(RigidBodyTransform transformToPack)
   {
      transformToPack.set(transformToCamera);
   }

   public void getOffsetTransform(RigidBodyTransformBasics offsetTransformToPack)
   {
      offsetTransformToPack.set(offsetTransform);
   }

   public void setOffset(RigidBodyTransformReadOnly newOffsetTransform)
   {
      offsetTransform.set(newOffsetTransform);
   }

   public void setRoll(double roll)
   {
      if (enablePanTiltRoll)
      {
         this.roll.set(roll);
      }
   }

   public void setPan(double pan)
   {
      if (enablePanTiltRoll)
      {
         this.pan.set(pan);
      }
   }

   public void setTilt(double tilt)
   {
      if (enablePanTiltRoll)
      {
         this.tilt.set(tilt);
      }
   }

   public void setFieldOfView(double fieldOfView)
   {
      this.fieldOfView = fieldOfView;
   }

   @Override
   public double getFieldOfView()
   {
      return fieldOfView;
   }

   @Override
   public double getClipDistanceNear()
   {
      return clipDistanceNear;
   }

   @Override
   public double getClipDistanceFar()
   {
      return clipDistanceFar;
   }

   @Override
   public void zoom(double amount)
   {
      fieldOfView = fieldOfView + amount;

      if (fieldOfView < 0.01)
      {
         fieldOfView = 0.01;
      }
      else if (fieldOfView > 3.0)
      {
         fieldOfView = 3.0;
      }

      //      System.out.println("Zoom. Field of View = " + fieldOfView);
   }

   public int getImageWidth()
   {
      return imageWidth;
   }

   public int getImageHeight()
   {
      return imageHeight;
   }

   public void setImageWidth(int imageWidth)
   {
      this.imageWidth = imageWidth;
   }

   public void setImageHeight(int imageHeight)
   {
      this.imageHeight = imageHeight;
   }
}
