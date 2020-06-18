package us.ihmc.simulationconstructionset.simulatedSensors;

import org.ejml.data.DMatrixRMaj;

import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.simulationconstructionset.Joint;

public interface WrenchCalculatorInterface
{
   String getName();

   void getTransformToParentJoint(RigidBodyTransform transformToPack);

   void calculate();

   DMatrixRMaj getWrench();

   Joint getJoint();

   void corruptWrenchElement(int row, double value);

   void setDoWrenchCorruption(boolean value);

}