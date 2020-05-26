package us.ihmc.simulationconstructionset;

import us.ihmc.euclid.matrix.Matrix3D;
import us.ihmc.euclid.matrix.interfaces.Matrix3DReadOnly;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple3D.interfaces.Tuple3DReadOnly;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoVector3D;

/**
 * This class implements a constraint for enforcing the closure of a kinematic loop.
 * <p>
 * This implementation uses a simple PD-control scheme to correct for violation of the loop closure.
 * </p>
 * 
 * @author Sylvain Bertrand
 */
public class LoopClosureSoftConstraint
{
   private String name;

   private final YoVector3D proportionalGains;
   private final YoVector3D derivativeGains;
   private final ExternalForcePoint constraintA;
   private final ExternalForcePoint constraintB;
   private final YoDouble positionErrorMagnitude;
   private final YoDouble rotationErrorMagnitude;

   private final Vector3D offsetFromParentJoint = new Vector3D();
   private final Vector3D offsetFromLinkParentJoint = new Vector3D();
   private final Matrix3D constraintForceSubSpace = new Matrix3D();
   private final Matrix3D constraintMomentSubSpace = new Matrix3D();

   private Joint parentJoint;

   private final Vector3D positionError = new Vector3D();
   private final Quaternion quaternionA = new Quaternion();
   private final Quaternion quaternionB = new Quaternion();
   private final Quaternion quaternionDifference = new Quaternion();
   private final Vector3D rotationError = new Vector3D();
   private final Vector3D linearVelocityError = new Vector3D();
   private final Vector3D angularVelocityError = new Vector3D();
   private final Vector3D forceA = new Vector3D();
   private final Vector3D momentA = new Vector3D();

   /**
    * Creates a new constraint for closing a kinematic loop.
    * <p>
    * This constraint is general and the sub-space in which it operates has to be provided. Here's a
    * couple examples:
    * <ul>
    * <li>For a constraint that only allows rotation around the z-axis, the matrices defining the
    * sub-space should be as follows:
    * 
    * <pre>
    *                           / 1 0 0 \
    * constraintForceSubSpace = | 0 1 0 |
    *                           \ 0 0 1 /
    *                            / 1 0 0 \
    * constraintMomentSubSpace = | 0 1 0 |
    *                            \ 0 0 0 /
    * </pre>
    * 
    * Note that by having <tt>constraintForceSubSpace</tt> be identity, the entire linear space is
    * constrained, while by having the last row of <tt>constraintMomentSubSpace</tt> be only zeros, the
    * z-axis is not constrained.
    * <li>For a constraint that only allows translation along the y-axis, the matrices defining the
    * sub-space should be as follows:
    * 
    * <pre>
    *                           / 1 0 0 \
    * constraintForceSubSpace = | 0 0 0 |
    *                           \ 0 0 1 /
    *                            / 1 0 0 \
    * constraintMomentSubSpace = | 0 1 0 |
    *                            \ 0 0 1 /
    * </pre>
    * </ul>
    * In other words, these matrices can be seen as selection matrices used for selecting which forces
    * and moments are to be applied.
    * </p>
    * 
    * @param name                      the name of the constraint, {@code YoVariable}s will be created
    *                                  using this name.
    * @param offsetFromParentJoint     the position of the constraint with respect to the parent joint.
    * @param offsetFromLinkParentJoint the position of the constraint with respect to the parent joint
    *                                  of the associated link. Note that the link's parent joint is
    *                                  expected to be different from this constraint's parent joint.
    * @param robot                     the robot is used for getting access to its
    *                                  {@code YoVariableRegistry}.
    * @param constraintForceSubSpace   defines the linear part of the sub-space in which the constraint
    *                                  is to be enforced.
    * @param constraintMomentSubSpace  defines the angular part of the sub-space in which the
    *                                  constraint is to be enforced.
    */
   public LoopClosureSoftConstraint(String name, Tuple3DReadOnly offsetFromParentJoint, Tuple3DReadOnly offsetFromLinkParentJoint, Robot robot,
                                    Matrix3DReadOnly constraintForceSubSpace, Matrix3DReadOnly constraintMomentSubSpace)
   {
      this.name = name;
      this.offsetFromParentJoint.set(offsetFromParentJoint);
      this.offsetFromLinkParentJoint.set(offsetFromLinkParentJoint);
      this.constraintForceSubSpace.set(constraintForceSubSpace);
      this.constraintMomentSubSpace.set(constraintMomentSubSpace);

      YoVariableRegistry registry = robot.getRobotsYoVariableRegistry();
      proportionalGains = new YoVector3D(name + "ProportionalGain", registry);
      derivativeGains = new YoVector3D(name + "DerivativeGain", registry);
      constraintA = new ExternalForcePoint(name + "A", offsetFromParentJoint, robot);
      constraintB = new ExternalForcePoint(name + "B", offsetFromLinkParentJoint, robot);
      positionErrorMagnitude = new YoDouble(name + "PositionErrorMagnitude", registry);
      rotationErrorMagnitude = new YoDouble(name + "RotationErrorMagnitude", registry);
   }

   public void setGains(double proportionalGain, double derivativeGain)
   {
      proportionalGains.set(proportionalGain, proportionalGain, proportionalGain);
      derivativeGains.set(derivativeGain, derivativeGain, derivativeGain);
   }

   /**
    * Sets the gains to use for enforcing this constraint.
    * <p>
    * Note that the gains are applied on the error in the local coordinates of the parent joints.
    * </p>
    * 
    * @param proportionalGains the gains to apply on the position and rotation errors.
    * @param derivativeGains   the gains to apply on the linear and angular velocity errors.
    */
   public void setGains(Tuple3DReadOnly proportionalGains, Tuple3DReadOnly derivativeGains)
   {
      this.proportionalGains.set(proportionalGains);
      this.derivativeGains.set(derivativeGains);
   }

   public void setParentJoint(Joint parentJoint)
   {
      this.parentJoint = parentJoint;
      parentJoint.addExternalForcePoint(constraintA);
   }

   public void setLink(Link link)
   {
      link.getParentJoint().addExternalForcePoint(constraintB);
   }

   public void update()
   {
      // Position error in world
      positionError.sub(constraintB.getYoPosition(), constraintA.getYoPosition());
      // Position error in A's local coordinates.
      parentJoint.transformToNext.inverseTransform(positionError);
      // Applying the sub-space on the error so the visualization is accurate, i.e. doesn't incorporate error that does not matter.
      constraintForceSubSpace.transform(positionError);
      positionErrorMagnitude.set(positionError.length());

      constraintA.getParentJoint().getRotationToWorld(quaternionA);
      constraintB.getParentJoint().getRotationToWorld(quaternionB);
      // Rotation error in A's local coordinates.
      quaternionDifference.difference(quaternionA, quaternionB); // This the orientation from B to A
      quaternionDifference.getRotationVector(rotationError);
      constraintMomentSubSpace.transform(rotationError);
      rotationErrorMagnitude.set(rotationError.length());

      // Linear velocity error in world
      linearVelocityError.sub(constraintB.getYoVelocity(), constraintA.getYoVelocity());
      // Linear velocity error in A's local coordinates.
      parentJoint.transformToNext.inverseTransform(linearVelocityError);
      constraintForceSubSpace.transform(linearVelocityError);

      // Angular velocity in world
      angularVelocityError.sub(constraintB.getYoAngularVelocity(), constraintA.getYoAngularVelocity());
      // Angular velocity error in A's local coordinates.
      parentJoint.transformToNext.inverseTransform(angularVelocityError);
      constraintMomentSubSpace.transform(angularVelocityError);

      positionError.scale(proportionalGains.getX(), proportionalGains.getY(), proportionalGains.getZ());
      rotationError.scale(proportionalGains.getX(), proportionalGains.getY(), proportionalGains.getZ());
      linearVelocityError.scale(derivativeGains.getX(), derivativeGains.getY(), derivativeGains.getZ());
      angularVelocityError.scale(derivativeGains.getX(), derivativeGains.getY(), derivativeGains.getZ());

      forceA.add(positionError, linearVelocityError);
      momentA.add(rotationError, angularVelocityError);

      // Need to switch back to world before applying these.
      parentJoint.transformToNext.transform(forceA);
      parentJoint.transformToNext.transform(momentA);

      constraintA.getYoForce().set(forceA);
      constraintA.getYoMoment().set(momentA);

      constraintB.getYoForce().setAndNegate(forceA);
      constraintB.getYoMoment().setAndNegate(momentA);
   }

   public String getName()
   {
      return name;
   }
}
