package us.ihmc.simulationconstructionset.simulatedSensors;

import java.util.HashMap;
import java.util.List;
import java.util.Map;

import org.ejml.data.DMatrixRMaj;

import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.simulationconstructionset.GroundContactPoint;
import us.ihmc.simulationconstructionset.Joint;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFrameVector3D;
import us.ihmc.yoVariables.registry.YoRegistry;

public class GroundContactPointBasedWrenchCalculator implements WrenchCalculatorInterface
{
   private static final int WRENCH_SIZE = 6;

   private final String forceSensorName;
   private final List<GroundContactPoint> contactPoints;
   private final Joint forceTorqueSensorJoint;

   private final RigidBodyTransform transformToParentJoint;

   private boolean doWrenchCorruption = false;
   private final DMatrixRMaj wrenchMatrix = new DMatrixRMaj(WRENCH_SIZE, 1);
   private final DMatrixRMaj corruptionMatrix = new DMatrixRMaj(WRENCH_SIZE, 1);
   private final Map<String, YoFrameVector3D> yoContactForceInSensorFrame = new HashMap<>();
   private final ReferenceFrame sensorFrame;

   public GroundContactPointBasedWrenchCalculator(String forceSensorName, List<GroundContactPoint> contactPoints, Joint forceTorqueSensorJoint,
                                                  RigidBodyTransform transformToParentJoint, YoRegistry registry)
   {
      this.forceSensorName = forceSensorName;
      this.contactPoints = contactPoints;
      this.forceTorqueSensorJoint = forceTorqueSensorJoint;
      this.transformToParentJoint = new RigidBodyTransform(transformToParentJoint);
      sensorFrame = new ReferenceFrame(forceSensorName + "Frame", ReferenceFrame.getWorldFrame())
      {
         @Override
         protected void updateTransformToParent(RigidBodyTransform transformToParent)
         {
            transformToParent.set(transformToOriginFrame);
         }
      };

      for (int i = 0; i < this.contactPoints.size(); i++)
      {
         if (registry != null)
         {
            String contactPointName = contactPoints.get(i).getName();
            String namePrefix = contactPointName + "_ForceInSensorFrame";
            // Checking if that variable does already exist for some reason.
            if (registry.findVariable(namePrefix + "X") == null)
               yoContactForceInSensorFrame.put(contactPointName, new YoFrameVector3D(namePrefix, sensorFrame, registry));
         }
      }
   }

   @Override
   public String getName()
   {
      return forceSensorName;
   }

   private final RigidBodyTransform transformToOriginFrame = new RigidBodyTransform();
   private final Vector3D force = new Vector3D();
   private final Vector3D contactPointMoment = new Vector3D();
   private final Point3D contactPointOriginFrame = new Point3D();
   private final Vector3D contactVectorOriginFrame = new Vector3D();
   private final Vector3D tau = new Vector3D();

   private final Point3D tempContactPoint = new Point3D();

   @Override
   public void calculate()
   {
      //OriginaFrame : sensorFrame
      wrenchMatrix.zero();
      forceTorqueSensorJoint.getTransformToWorld(transformToOriginFrame);
      transformToOriginFrame.multiply(transformToParentJoint);
      sensorFrame.update();
      transformToOriginFrame.invert();

      for (int i = 0; i < contactPoints.size(); i++)
      {
         GroundContactPoint contactPoint = contactPoints.get(i);
         contactPoint.getForce(force);
         contactPoint.getMoment(contactPointMoment);

         transformToOriginFrame.transform(force);
         transformToOriginFrame.transform(contactPointMoment);

         contactPointOriginFrame.set(0.0, 0.0, 0.0);

         contactPoint.getPosition(tempContactPoint);
         transformToOriginFrame.transform(tempContactPoint, contactPointOriginFrame);
         contactVectorOriginFrame.set(contactPointOriginFrame);
         tau.cross(contactVectorOriginFrame, force);
         tau.add(contactPointMoment);

         wrenchMatrix.set(0, 0, wrenchMatrix.get(0, 0) + tau.getX());
         wrenchMatrix.set(1, 0, wrenchMatrix.get(1, 0) + tau.getY());
         wrenchMatrix.set(2, 0, wrenchMatrix.get(2, 0) + tau.getZ());

         wrenchMatrix.set(3, 0, wrenchMatrix.get(3, 0) + force.getX());
         wrenchMatrix.set(4, 0, wrenchMatrix.get(4, 0) + force.getY());
         wrenchMatrix.set(5, 0, wrenchMatrix.get(5, 0) + force.getZ());

         if (yoContactForceInSensorFrame.containsKey(contactPoint.getName()))
         {
            yoContactForceInSensorFrame.get(contactPoint.getName()).set(force);
         }
      }

      if (doWrenchCorruption)
      {
         for (int i = 0; i < WRENCH_SIZE; i++)
         {
            wrenchMatrix.add(i, 0, corruptionMatrix.get(i, 0));
         }
      }
   }

   @Override
   public Joint getJoint()
   {
      return forceTorqueSensorJoint;
   }

   @Override
   public DMatrixRMaj getWrench()
   {
      return wrenchMatrix;
   }

   @Override
   public void setDoWrenchCorruption(boolean value)
   {
      doWrenchCorruption = value;
   }

   @Override
   public void corruptWrenchElement(int row, double value)
   {
      corruptionMatrix.add(row, 0, value);
   }

   @Override
   public void getTransformToParentJoint(RigidBodyTransform transformToPack)
   {
      transformToPack.set(transformToParentJoint);
   }

   @Override
   public String toString()
   {
      return forceSensorName + " " + contactPoints;
   }
}
