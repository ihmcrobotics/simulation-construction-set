package us.ihmc.simulationconstructionset.simulatedSensors;

import java.util.List;

import org.ejml.data.DMatrixRMaj;

import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.simulationconstructionset.ExternalForcePoint;
import us.ihmc.simulationconstructionset.Joint;
import us.ihmc.yoVariables.registry.YoRegistry;

public class CollisionShapeBasedWrenchCalculator implements WrenchCalculatorInterface
{
   private static final int WRENCH_SIZE = 6;

   private final String forceSensorName;
   private final List<ExternalForcePoint> contactPoints;
   private final Joint forceTorqueSensorJoint;

   private final RigidBodyTransform transformToParentJoint;

   private boolean doWrenchCorruption = false;
   private final DMatrixRMaj wrenchMatrix = new DMatrixRMaj(WRENCH_SIZE, 1);
   private final DMatrixRMaj corruptionMatrix = new DMatrixRMaj(WRENCH_SIZE, 1);

   private final ReferenceFrame sensorFrame;

   private final RigidBodyTransform transformToOriginFrame = new RigidBodyTransform();
   private final Vector3D force = new Vector3D();
   private final Point3D contactPointOriginFrame = new Point3D();
   private final Vector3D contactVectorOriginFrame = new Vector3D();
   private final Vector3D tau = new Vector3D();

   public CollisionShapeBasedWrenchCalculator(String forceSensorName, List<ExternalForcePoint> contactPoints, Joint forceTorqueSensorJoint,
                                              RigidBodyTransform transformToParentJoint, YoRegistry registry)
   {
      this.forceSensorName = forceSensorName;
      this.contactPoints = contactPoints;
      this.forceTorqueSensorJoint = forceTorqueSensorJoint;
      this.transformToParentJoint = transformToParentJoint;

      sensorFrame = new ReferenceFrame(forceSensorName + "Frame", ReferenceFrame.getWorldFrame())
      {
         @Override
         protected void updateTransformToParent(RigidBodyTransform transformToParent)
         {
            transformToParent.set(transformToOriginFrame);
         }
      };
   }

   @Override
   public String getName()
   {
      return forceSensorName;
   }

   @Override
   public void getTransformToParentJoint(RigidBodyTransform transformToPack)
   {
      transformToPack.set(transformToParentJoint);
   }

   @Override
   public void calculate()
   {
      wrenchMatrix.zero();

      forceTorqueSensorJoint.getTransformToWorld(transformToOriginFrame);
      transformToOriginFrame.multiply(transformToParentJoint);
      sensorFrame.update();
      transformToOriginFrame.invert();

      for (int i = 0; i < contactPoints.size(); i++)
      {
         ExternalForcePoint contactPoint = contactPoints.get(i);
         Point3D tempContactPoint = new Point3D();

         contactPoint.getForce(force);

         wrenchMatrix.set(3, 0, wrenchMatrix.get(3, 0) + force.getX());
         wrenchMatrix.set(4, 0, wrenchMatrix.get(4, 0) + force.getY());
         wrenchMatrix.set(5, 0, wrenchMatrix.get(5, 0) + force.getZ());

         transformToOriginFrame.transform(force);

         contactPointOriginFrame.set(0.0, 0.0, 0.0);

         contactPoint.getPosition(tempContactPoint);
         transformToOriginFrame.transform(tempContactPoint, contactPointOriginFrame);
         contactVectorOriginFrame.set(contactPointOriginFrame);
         tau.cross(contactVectorOriginFrame, force);

         wrenchMatrix.set(0, 0, wrenchMatrix.get(0, 0) + tau.getX());
         wrenchMatrix.set(1, 0, wrenchMatrix.get(1, 0) + tau.getY());
         wrenchMatrix.set(2, 0, wrenchMatrix.get(2, 0) + tau.getZ());
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
   public DMatrixRMaj getWrench()
   {
      return wrenchMatrix;
   }

   @Override
   public Joint getJoint()
   {
      return forceTorqueSensorJoint;
   }

   @Override
   public void corruptWrenchElement(int row, double value)
   {
      corruptionMatrix.add(row, 0, value);
   }

   @Override
   public void setDoWrenchCorruption(boolean value)
   {
      doWrenchCorruption = value;
   }

}
