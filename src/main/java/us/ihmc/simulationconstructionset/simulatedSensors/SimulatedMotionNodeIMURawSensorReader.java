package us.ihmc.simulationconstructionset.simulatedSensors;

import us.ihmc.euclid.matrix.RotationMatrix;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.robotics.math.corruptors.NoisyYoDouble;
import us.ihmc.robotics.math.corruptors.NoisyYoRotationMatrix;
import us.ihmc.robotics.screwTheory.RigidBody;
import us.ihmc.robotics.screwTheory.SpatialAccelerationVector;
import us.ihmc.robotics.sensors.RawIMUSensorsInterface;

public class SimulatedMotionNodeIMURawSensorReader extends SimulatedIMURawSensorReader
{
   private static final double GRAVITY = 9.80665; // MotionNode's definition of g, as given on p. 5 of 'MotionNode SDK Reference'

   // Data taken from http://www.motionnode.com/MotionNode_Specification.pdf
   public static final double DT = 0.01; // Sampling rate
   private static final boolean ORIENTATION_IS_NOISY = true;
   private static final double ORIENTATION_STANDARD_DEVIATION = 0.0; // output is filtered with an unknown bandwith
   private static final boolean ORIENTATION_USE_BIAS = true;
   private static final double ORIENTATION_BIAS_ROTATION_ANGLE_MAX = 1.0 / 180.0 * Math.PI; // for low dynamics: 1.0 / 180.0 * Math.PI, for high dynamics: 4.0 / 180.0 * Math.PI, TODO: make this number depending on the accelerometer values
   private static final double ORIENTATION_BIAS_ROTATION_ANGLE_MIN = -ORIENTATION_BIAS_ROTATION_ANGLE_MAX;
   private static final double ORIENTATION_BIAS_ROTATION_ANGLE_DELTA = 0.0002; // our choice for simulation
   private static final double ORIENTATION_BIAS_DIRECTION_HEIGHT_DELTA = 0.001; // our choice for simulation
   private static final double ORIENTATION_BIAS_DIRECTION_ANGLE_DELTA = ORIENTATION_BIAS_DIRECTION_HEIGHT_DELTA * Math.PI;

   private static final boolean ACCEL_IS_NOISY = true;
   private static final double ACCEL_STANDARD_DEVIATION = 0.1; // ?
   private static final boolean ACCEL_USE_BIAS = true;
   private static final double ACCEL_BIAS_MAX = 0.003 * GRAVITY; // depending on 2g or 6g range
   private static final double ACCEL_BIAS_MIN = -ACCEL_BIAS_MAX;
   private static final double ACCEL_BIAS_DELTA = ACCEL_BIAS_MAX / 100.0; // our choice for simulation

   private static final boolean GYRO_IS_NOISY = true;
   private static final double GYRO_STANDARD_DEVIATION = 0.05; // ?
   private static final boolean GYRO_USE_BIAS = true;
   private static final double GYRO_BIAS_MAX = Double.POSITIVE_INFINITY;
   private static final double GYRO_BIAS_MIN = -GYRO_BIAS_MAX;
   private static final double GYRO_BIAS_DELTA = 0.1 / 180.0 * Math.PI * DT;
   
   private final RotationMatrix imuMountingOffset;
   private final double localGravityZ;

   public SimulatedMotionNodeIMURawSensorReader(RawIMUSensorsInterface rawSensors, int imuIndex, RigidBody rigidBody, ReferenceFrame imuFrame,
         RigidBody rootBody, SpatialAccelerationVector rootAcceleration, RotationMatrix imuMountingOffset, double localGravityPositiveZ)
   {
      super(rawSensors, imuIndex, rigidBody, imuFrame, rootBody, rootAcceleration);
      
      this.imuMountingOffset = imuMountingOffset;
      this.localGravityZ = localGravityPositiveZ;
   }

   @Override
   protected void initializeNoise()
   {
      rotationMatrix.setGaussianNoise(ORIENTATION_STANDARD_DEVIATION);
      rotationMatrix.setBiasOfRotationAngle(NoisyYoRotationMatrix.DEFAULT_BIAS_ROTATION_ANGLE, ORIENTATION_BIAS_ROTATION_ANGLE_MAX, ORIENTATION_BIAS_ROTATION_ANGLE_MIN, ORIENTATION_BIAS_ROTATION_ANGLE_DELTA);
      rotationMatrix.setBiasOfDirectionHeight(NoisyYoRotationMatrix.DEFAULT_BIAS_DIRECTION_HEIGHT, NoisyYoRotationMatrix.DEFAULT_BIAS_DIRECTION_HEIGHT_MAX, NoisyYoRotationMatrix.DEFAULT_BIAS_DIRECTION_HEIGHT_MIN, ORIENTATION_BIAS_DIRECTION_HEIGHT_DELTA);
      rotationMatrix.setBiasOfDirectionAngle(NoisyYoRotationMatrix.DEFAULT_BIAS_DIRECTION_ANGLE, NoisyYoRotationMatrix.DEFAULT_BIAS_DIRECTION_ANGLE_MAX, NoisyYoRotationMatrix.DEFAULT_BIAS_DIRECTION_ANGLE_MIN, ORIENTATION_BIAS_DIRECTION_ANGLE_DELTA);
      rotationMatrix.setBiasRandomlyBetweenMinAndMax();
      rotationMatrix.setBias(ORIENTATION_USE_BIAS);
      rotationMatrix.setIsNoisy(ORIENTATION_IS_NOISY);

      initializeGaussianNoise(accelList, ACCEL_IS_NOISY, ACCEL_STANDARD_DEVIATION, ACCEL_USE_BIAS, 0.0, ACCEL_BIAS_MAX, ACCEL_BIAS_MIN, ACCEL_BIAS_DELTA, true);
      initializeGaussianNoise(gyroList, GYRO_IS_NOISY, GYRO_STANDARD_DEVIATION, GYRO_USE_BIAS, 0.0, GYRO_BIAS_MAX, GYRO_BIAS_MIN, GYRO_BIAS_DELTA, false);
      // TODO add compass
   }

   // Make sure that simulateIMU() is only executed at original sample rate!
   @Override
   protected void simulateIMU()
   {
      // TODO: Add time delay, internal filters, sensor range clipping, nonlinearities

      rotationMatrix.update(perfM00.getDoubleValue(), perfM01.getDoubleValue(), perfM02.getDoubleValue(), perfM10.getDoubleValue(), perfM11.getDoubleValue(),
            perfM12.getDoubleValue(), perfM20.getDoubleValue(), perfM21.getDoubleValue(), perfM22.getDoubleValue());
      
      //12112012 added mounting orientation offset.
      RotationMatrix noisyRotationInWorld = rotationMatrix.getMatrix3d();
      noisyRotationInWorld.multiplyTransposeOther(imuMountingOffset);
      
      accelX.update(perfAccelX.getDoubleValue());
      accelY.update(perfAccelY.getDoubleValue());
      accelZ.update(perfAccelZ.getDoubleValue());
      
      //12112012 Acceleration output in worldFrame instead of imu frame
      FrameVector3D noisyAcceleration = new FrameVector3D(imuFrame, accelX.getDoubleValue(), accelY.getDoubleValue(), accelZ.getDoubleValue());
      noisyAcceleration.changeFrame(worldFrame);
      
      //12112012 Acceleration subtract gravity
      noisyAcceleration.sub(new FrameVector3D(worldFrame, 0.0, 0.0, localGravityZ));
      
      accelX.set(noisyAcceleration.getX());
      accelY.set(noisyAcceleration.getY());
      accelZ.set(noisyAcceleration.getZ());
      
      
      gyroX.update(perfGyroX.getDoubleValue());
      gyroY.update(perfGyroY.getDoubleValue());
      gyroZ.update(perfGyroZ.getDoubleValue());

      compassX.update(perfCompassX.getDoubleValue());
      compassY.update(perfCompassY.getDoubleValue());
      compassZ.update(perfCompassZ.getDoubleValue());
   }

   private void initializeGaussianNoise(NoisyYoDouble[] list, boolean isNoisy, double standardDeviation, boolean useBias, double bias, double biasMax,
         double biasMin, double biasDelta, boolean setBiasRandomlyBetweenMinAndMax)
   {
      for (NoisyYoDouble i : list)
      {
         i.setIsNoisy(isNoisy);
         i.setGaussianNoise(standardDeviation);

         if (useBias)
         {
            i.setBias(bias, biasMax, biasMin, biasDelta);
         }
         else
         {
            i.setBias(false);
         }
         
         if (setBiasRandomlyBetweenMinAndMax)
         {
            i.setBiasRandomlyBetweenMinAndMax();
         }
      }
   }
}