package us.ihmc.simulationconstructionset;

import us.ihmc.euclid.matrix.RotationMatrix;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFrameQuaternion;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFrameVector3D;
import us.ihmc.yoVariables.registry.YoRegistry;

public class IMUMount
{
   private final String name;

   private RigidBodyTransform transformFromMountToJoint;

   private Joint parentJoint;

   private Robot robot;

   private final YoFrameVector3D angularVelocityInBody;
   private final YoFrameVector3D angularAccelerationInBody;

   private final YoFrameVector3D linearVelocityInBody;
   private final YoFrameVector3D linearVelocityInWorld;

   private final YoFrameVector3D linearAccelerationInBody;
   private final YoFrameVector3D linearAccelerationInWorld;

   private final YoFrameQuaternion orientation;

   private double accelerationGaussianNoiseMean = 0.0, accelerationGaussianNoiseStdDev = 0.0;
   private double accelerationGaussianBiasMean = 0.0, accelerationGaussianBiasStdDev = 0.0;

   private double angularVelocityGaussianNoiseMean = 0.0, angularVelocityGaussianNoiseStdDev = 0.0;
   private double angularVelocityGaussianBiasMean = 0.0, angularVelocityGaussianBiasStdDev = 0.0;

   public IMUMount(String name, RigidBodyTransform offset, Robot robot)
   {
      this.name = name;
      this.robot = robot;

      transformFromMountToJoint = new RigidBodyTransform(offset);

      YoRegistry registry = robot.getRobotsYoRegistry();

      orientation = new YoFrameQuaternion(name + "Orientation", null, registry);

      linearVelocityInBody = new YoFrameVector3D(name + "LinearVelocity", null, registry);
      linearVelocityInWorld = new YoFrameVector3D(name + "LinearVelocityWorld", null, registry);

      angularVelocityInBody = new YoFrameVector3D(name + "AngularVelocity", null, registry);
      angularAccelerationInBody = new YoFrameVector3D(name + "AngularAcceleration", null, registry);

      linearAccelerationInBody = new YoFrameVector3D(name + "LinearAcceleration", null, registry);
      linearAccelerationInWorld = new YoFrameVector3D(name + "LinearAccelerationWorld", null, registry);
   }

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

   private final RigidBodyTransform imuTransformToWorld = new RigidBodyTransform();

   protected void updateIMUMountPositionAndVelocity()
   {
      parentJoint.getTransformToWorld(imuTransformToWorld);
      imuTransformToWorld.multiply(transformFromMountToJoint);

      // Orientation:
      orientation.set(imuTransformToWorld.getRotation());

      // TODO: These are the values stored from whatever the last stage of the integrator did.
      // They do not get averaged with the RungeKutta (or other integrator) averager.

      // Linear Velocity
      parentJoint.physics.getLinearVelocityInBody(linearVelocityInBody, transformFromMountToJoint.getTranslation());
      transformFromMountToJoint.inverseTransform(linearVelocityInBody); // Joint frame => IMU frame.
      imuTransformToWorld.transform(linearVelocityInBody, linearVelocityInWorld); // IMU frame => world

      // Angular Velocity
      parentJoint.physics.getAngularVelocityInBody(angularVelocityInBody);
      transformFromMountToJoint.inverseTransform(angularVelocityInBody); // Joint frame => IMU frame
   }

   private final Vector3D tempGravity = new Vector3D();

   protected void updateIMUMountAcceleration()
   {
      if (name.contains("pelvisMiddleImu"))
      {
         System.out.print("");
      }
      // We redo some of the things that are already done in updateIMUMountPositionAndVelocity,
      // but it is safer that way since updateIMUMountAcceleration might be called by itself sometimes.
      parentJoint.getTransformToWorld(imuTransformToWorld);
      imuTransformToWorld.multiply(transformFromMountToJoint);

      // Linear Acceleration
      parentJoint.physics.getLinearAccelerationInBody(linearAccelerationInBody, transformFromMountToJoint.getTranslation());
      transformFromMountToJoint.inverseTransform(linearAccelerationInBody); // Joint frame => IMU frame

      robot.getGravity(tempGravity);
      tempGravity.scale(-1.0);

      imuTransformToWorld.inverseTransform(tempGravity); // World => IMU frame
      linearAccelerationInBody.add(tempGravity);

      parentJoint.physics.getLinearAccelerationInWorld(linearAccelerationInWorld, transformFromMountToJoint.getTranslation());
      robot.getGravity(tempGravity);
      tempGravity.scale(-1.0);
      linearAccelerationInWorld.add(tempGravity);

      // Angular Acceleration
      parentJoint.physics.getAngularAccelerationsInBodyFrame(angularAccelerationInBody);
      transformFromMountToJoint.inverseTransform(angularAccelerationInBody); // Joint frame => IMU frame
   }

   public void setOrientation(Quaternion orientation)
   {
      this.orientation.set(orientation);
   }

   public void getOrientation(Quaternion orientationToPack)
   {
      orientationToPack.set(orientation);
   }

   public void getOrientation(RotationMatrix rotationMatrixToPack)
   {
      rotationMatrixToPack.set(orientation);
   }

   public void setAngularVelocityInBody(Vector3D angularVelocityInBody)
   {
      this.angularVelocityInBody.set(angularVelocityInBody);
   }

   public void getAngularVelocityInBody(Vector3D angularVelocityInBodyToPack)
   {
      angularVelocityInBodyToPack.set(angularVelocityInBody);
   }

   public void setAngularAccelerationInBody(Vector3D angularAccelerationInBody)
   {
      this.angularAccelerationInBody.set(angularAccelerationInBody);
   }

   public void getAngularAccelerationInBody(Vector3D angularAccelerationInBodyToPack)
   {
      angularAccelerationInBodyToPack.set(angularAccelerationInBody);
   }

   public void setLinearAccelerationInBody(Vector3D linearAccelerationInBody)
   {
      this.linearAccelerationInBody.set(linearAccelerationInBody);
   }

   public void getLinearAccelerationInBody(Vector3D linearAccelerationInBodyToPack)
   {
      linearAccelerationInBodyToPack.set(linearAccelerationInBody);
   }

   public void getTransformFromMountToJoint(RigidBodyTransform transformToPack)
   {
      transformToPack.set(transformFromMountToJoint);
   }

   public void setOffset(RigidBodyTransform newTransformFromMountToJoint)
   {
      transformFromMountToJoint.set(newTransformFromMountToJoint);
   }

   public void setAngularVelocityNoiseParameters(double angularVelocityGaussianNoiseMean, double angularVelocityGaussianNoiseStdDev)
   {
      this.angularVelocityGaussianNoiseMean = angularVelocityGaussianNoiseMean;
      this.angularVelocityGaussianNoiseStdDev = angularVelocityGaussianNoiseStdDev;
   }

   public void setAngularVelocityBiasParameters(double angularVelocityGaussianBiasMean, double angularVelocityGaussianBiasStdDev)
   {
      this.angularVelocityGaussianBiasMean = angularVelocityGaussianBiasMean;
      this.angularVelocityGaussianBiasStdDev = angularVelocityGaussianBiasStdDev;
   }

   public void setAccelerationNoiseParameters(double accelerationGaussianNoiseMean, double accelerationGaussianNoiseStdDev)
   {
      this.accelerationGaussianNoiseMean = accelerationGaussianNoiseMean;
      this.accelerationGaussianNoiseStdDev = accelerationGaussianNoiseStdDev;
   }

   public void setAccelerationBiasParameters(double accelerationGaussianBiasMean, double accelerationGaussianBiasStdDev)
   {
      this.accelerationGaussianBiasMean = accelerationGaussianBiasMean;
      this.accelerationGaussianBiasStdDev = accelerationGaussianBiasStdDev;
   }

   public double getAccelerationGaussianNoiseMean()
   {
      return accelerationGaussianNoiseMean;
   }

   public double getAccelerationGaussianNoiseStdDev()
   {
      return accelerationGaussianNoiseStdDev;
   }

   public double getAccelerationGaussianBiasMean()
   {
      return accelerationGaussianBiasMean;
   }

   public double getAccelerationGaussianBiasStdDev()
   {
      return accelerationGaussianBiasStdDev;
   }

   public double getAngularVelocityGaussianNoiseMean()
   {
      return angularVelocityGaussianNoiseMean;
   }

   public double getAngularVelocityGaussianNoiseStdDev()
   {
      return angularVelocityGaussianNoiseStdDev;
   }

   public double getAngularVelocityGaussianBiasMean()
   {
      return angularVelocityGaussianBiasMean;
   }

   public double getAngularVelocityGaussianBiasStdDev()
   {
      return angularVelocityGaussianBiasStdDev;
   }

}
