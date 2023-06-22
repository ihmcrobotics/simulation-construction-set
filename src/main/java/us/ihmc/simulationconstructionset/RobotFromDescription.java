package us.ihmc.simulationconstructionset;

import java.util.ArrayList;
import java.util.Collection;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;
import java.util.Objects;

import us.ihmc.euclid.matrix.interfaces.Matrix3DReadOnly;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple3D.interfaces.Tuple3DReadOnly;
import us.ihmc.graphicsDescription.conversion.VisualsConversionTools;
import us.ihmc.robotics.robotDescription.BallAndSocketJointDescription;
import us.ihmc.robotics.robotDescription.CameraSensorDescription;
import us.ihmc.robotics.robotDescription.CollisionMeshDescription;
import us.ihmc.robotics.robotDescription.ExternalForcePointDescription;
import us.ihmc.robotics.robotDescription.FloatingJointDescription;
import us.ihmc.robotics.robotDescription.FloatingPlanarJointDescription;
import us.ihmc.robotics.robotDescription.ForceSensorDescription;
import us.ihmc.robotics.robotDescription.GroundContactPointDescription;
import us.ihmc.robotics.robotDescription.IMUSensorDescription;
import us.ihmc.robotics.robotDescription.JointDescription;
import us.ihmc.robotics.robotDescription.JointWrenchSensorDescription;
import us.ihmc.robotics.robotDescription.KinematicPointDescription;
import us.ihmc.robotics.robotDescription.LidarSensorDescription;
import us.ihmc.robotics.robotDescription.LinkDescription;
import us.ihmc.robotics.robotDescription.LinkGraphicsDescription;
import us.ihmc.robotics.robotDescription.LoopClosureConstraintDescription;
import us.ihmc.robotics.robotDescription.PinJointDescription;
import us.ihmc.robotics.robotDescription.Plane;
import us.ihmc.robotics.robotDescription.RobotDescription;
import us.ihmc.robotics.robotDescription.SliderJointDescription;
import us.ihmc.scs2.definition.robot.CameraSensorDefinition;
import us.ihmc.scs2.definition.robot.ExternalWrenchPointDefinition;
import us.ihmc.scs2.definition.robot.FixedJointDefinition;
import us.ihmc.scs2.definition.robot.GroundContactPointDefinition;
import us.ihmc.scs2.definition.robot.IMUSensorDefinition;
import us.ihmc.scs2.definition.robot.JointDefinition;
import us.ihmc.scs2.definition.robot.KinematicPointDefinition;
import us.ihmc.scs2.definition.robot.LidarSensorDefinition;
import us.ihmc.scs2.definition.robot.LoopClosureDefinition;
import us.ihmc.scs2.definition.robot.PlanarJointDefinition;
import us.ihmc.scs2.definition.robot.PrismaticJointDefinition;
import us.ihmc.scs2.definition.robot.RevoluteJointDefinition;
import us.ihmc.scs2.definition.robot.RigidBodyDefinition;
import us.ihmc.scs2.definition.robot.RobotDefinition;
import us.ihmc.scs2.definition.robot.SixDoFJointDefinition;
import us.ihmc.scs2.definition.robot.WrenchSensorDefinition;
import us.ihmc.simulationconstructionset.simulatedSensors.CollisionShapeBasedWrenchCalculator;
import us.ihmc.simulationconstructionset.simulatedSensors.FeatherStoneJointBasedWrenchCalculator;
import us.ihmc.simulationconstructionset.simulatedSensors.GroundContactPointBasedWrenchCalculator;
import us.ihmc.simulationconstructionset.simulatedSensors.LidarMount;
import us.ihmc.simulationconstructionset.simulatedSensors.WrenchCalculatorInterface;

public class RobotFromDescription extends Robot implements OneDegreeOfFreedomJointHolder
{
   private final Map<String, Joint> jointNameMap = new LinkedHashMap<>();

   private final Map<String, OneDegreeOfFreedomJoint> oneDegreeOfFreedomJoints = new LinkedHashMap<>();

   private final Map<String, CameraMount> cameraNameMap = new LinkedHashMap<>();
   private final Map<String, LidarMount> lidarNameMap = new LinkedHashMap<>();
   private final Map<String, IMUMount> imuNameMap = new LinkedHashMap<>();
   private final Map<String, JointWrenchSensor> wrenchSensorNameMap = new LinkedHashMap<>();

   private final Map<Joint, ArrayList<GroundContactPoint>> jointToGroundContactPointsMap = new LinkedHashMap<>();

   public RobotFromDescription(RobotDescription description)
   {
      this(description, true, true);
   }

   public RobotFromDescription(RobotDescription description, boolean enableDamping, boolean enableJointTorqueAndVelocityLimits)
   {
      super(description.getName());
      constructRobotFromDescription(description, enableDamping, enableJointTorqueAndVelocityLimits);
   }

   public RobotFromDescription(RobotDefinition definition)
   {
      this(definition, true, true);
   }

   public RobotFromDescription(RobotDefinition definition, boolean enableDamping, boolean enableJointTorqueAndVelocityLimits)
   {
      super(definition.getName());
      constructRobotFromDefinition(definition, enableDamping, enableJointTorqueAndVelocityLimits);
   }

   @Override
   public Joint getJoint(String jointName)
   {
      return jointNameMap.get(jointName);
   }

   @Override
   public OneDegreeOfFreedomJoint getOneDegreeOfFreedomJoint(String name)
   {
      return oneDegreeOfFreedomJoints.get(name);
   }

   public OneDegreeOfFreedomJoint[] getOneDegreeOfFreedomJoints()
   {
      return oneDegreeOfFreedomJoints.values().toArray(new OneDegreeOfFreedomJoint[oneDegreeOfFreedomJoints.size()]);
   }

   public CameraMount getCameraMount(String cameraName)
   {
      return cameraNameMap.get(cameraName);
   }

   public IMUMount getIMUMount(String name)
   {
      return imuNameMap.get(name);
   }

   public JointWrenchSensor getJointWrenchSensor(String name)
   {
      return wrenchSensorNameMap.get(name);
   }

   public ArrayList<GroundContactPoint> getGroundContactPointsOnJoint(Joint joint)
   {
      return jointToGroundContactPointsMap.get(joint);
   }

   private void constructRobotFromDescription(RobotDescription description, boolean enableDamping, boolean enableJointTorqueAndVelocityLimits)
   {
      List<JointDescription> rootJointDescriptions = description.getRootJoints();

      for (JointDescription rootJointDescription : rootJointDescriptions)
      {
         Joint rootJoint = constructJointRecursively(rootJointDescription, enableDamping, enableJointTorqueAndVelocityLimits);
         addRootJoint(rootJoint);
      }

      for (JointDescription rootJointDescription : rootJointDescriptions)
      {
         addLoopClosureConstraintsRecursively(rootJointDescription);
      }

      for (JointDescription rootJointDescription : rootJointDescriptions)
      {
         addForceSensorRecursively(rootJointDescription);
      }
   }

   private Joint constructJointRecursively(JointDescription jointDescription, boolean enableDamping, boolean enableJointTorqueAndVelocityLimits)
   {
      Joint joint = createSingleJoint(jointDescription, enableDamping, enableJointTorqueAndVelocityLimits);

      addGroundContactPoints(jointDescription, joint);
      addExternalForcePoints(jointDescription, joint);
      addKinematicPoints(jointDescription, joint);

      addExternalForcePointsFromCollisionMesh(jointDescription, joint);

      addLidarMounts(jointDescription, joint);
      addCameraMounts(jointDescription, joint);
      addIMUMounts(jointDescription, joint);
      addJointWrenchSensors(jointDescription, joint);

      //addForceSensors(jointDescription, joint);

      // Iterate over the children
      List<JointDescription> childrenJoints = jointDescription.getChildrenJoints();
      for (JointDescription childJointDescription : childrenJoints)
      {
         Joint childJoint = constructJointRecursively(childJointDescription, enableDamping, enableJointTorqueAndVelocityLimits);
         joint.addJoint(childJoint);
      }

      jointNameMap.put(joint.getName(), joint);

      if (joint instanceof OneDegreeOfFreedomJoint)
      {
         oneDegreeOfFreedomJoints.put(joint.getName(), (OneDegreeOfFreedomJoint) joint);
      }

      return joint;
   }

   private void addForceSensorRecursively(JointDescription jointDescription)
   {
      Joint joint = jointNameMap.get(jointDescription.getName());

      List<ForceSensorDescription> forceSensorDescriptions = jointDescription.getForceSensors();

      for (ForceSensorDescription forceSensorDescription : forceSensorDescriptions)
      {
         WrenchCalculatorInterface wrenchCalculator;

         if (forceSensorDescription.useGroundContactPoints())
         {
            ArrayList<GroundContactPoint> groundContactPoints = new ArrayList<>();
            joint.recursiveGetAllGroundContactPoints(groundContactPoints);

            wrenchCalculator = new GroundContactPointBasedWrenchCalculator(forceSensorDescription.getName(),
                                                                           groundContactPoints,
                                                                           joint,
                                                                           forceSensorDescription.getTransformToJoint(),
                                                                           yoRegistry);

            if (forceSensorDescription.useShapeCollision())
            {
               List<ExternalForcePoint> contactPoints = new ArrayList<>();
               contactPoints = joint.getExternalForcePoints();
               wrenchCalculator = new CollisionShapeBasedWrenchCalculator(forceSensorDescription.getName(),
                                                                          contactPoints,
                                                                          joint,
                                                                          forceSensorDescription.getTransformToJoint(),
                                                                          yoRegistry);
            }
         }
         else
         {
            Vector3D offsetToPack = new Vector3D();
            offsetToPack.set(forceSensorDescription.getTransformToJoint().getTranslation());
            JointWrenchSensor jointWrenchSensor = new JointWrenchSensor(forceSensorDescription.getName(), offsetToPack, this);
            joint.addJointWrenchSensor(jointWrenchSensor);

            wrenchCalculator = new FeatherStoneJointBasedWrenchCalculator(forceSensorDescription.getName(), joint);
         }

         joint.addForceSensor(wrenchCalculator);
      }

      List<JointDescription> childrenJoints = jointDescription.getChildrenJoints();

      for (JointDescription childJointDescription : childrenJoints)
         addForceSensorRecursively(childJointDescription);
   }

   private void addLoopClosureConstraintsRecursively(JointDescription jointDescription)
   {
      Joint joint = jointNameMap.get(jointDescription.getName());

      List<LoopClosureConstraintDescription> constraintDescriptions = jointDescription.getChildrenConstraintDescriptions();

      for (LoopClosureConstraintDescription constraintDescription : constraintDescriptions)
      {
         String name = constraintDescription.getName();
         Tuple3DReadOnly offsetFromParentJoint = constraintDescription.getOffsetFromParentJoint();
         Tuple3DReadOnly offsetFromLinkParentJoint = constraintDescription.getOffsetFromLinkParentJoint();
         Matrix3DReadOnly constraintForceSubSpace = constraintDescription.getConstraintForceSubSpace();
         Matrix3DReadOnly constraintMomentSubSpace = constraintDescription.getConstraintMomentSubSpace();
         LoopClosureSoftConstraint constraint = new LoopClosureSoftConstraint(name,
                                                                              offsetFromParentJoint,
                                                                              offsetFromLinkParentJoint,
                                                                              this,
                                                                              constraintForceSubSpace,
                                                                              constraintMomentSubSpace);
         constraint.setGains(constraintDescription.getProportionalGains(), constraintDescription.getDerivativeGains());

         joint.addLoopClosureConstraint(constraint);
         Link link = getLink(constraintDescription.getLink().getName());
         Objects.requireNonNull(link, "Could not find link: " + constraintDescription.getLink().getName());
         constraint.setLink(link);
      }

      for (JointDescription childJointDescription : jointDescription.getChildrenJoints())
      {
         addLoopClosureConstraintsRecursively(childJointDescription);
      }
   }

   private void addExternalForcePointsFromCollisionMesh(JointDescription jointDescription, Joint joint)
   {
      Link link = joint.getLink();
      List<CollisionMeshDescription> collisionMeshDescriptions = link.getCollisionMeshDescriptions();

      if (collisionMeshDescriptions != null)
      {
         int estimatedNumberOfContactPoints = 0;

         for (int i = 0; i < collisionMeshDescriptions.size(); i++)
         {
            CollisionMeshDescription collisionMesh = collisionMeshDescriptions.get(i);
            estimatedNumberOfContactPoints += collisionMesh.getEstimatedNumberOfContactPoints();
         }

         link.enableContactingExternalForcePoints(estimatedNumberOfContactPoints, yoRegistry);
      }
   }

   private void addLidarMounts(JointDescription jointDescription, Joint joint)
   {
      List<LidarSensorDescription> lidarSensorDescriptions = jointDescription.getLidarSensors();

      for (LidarSensorDescription lidarSensorDescription : lidarSensorDescriptions)
      {
         LidarMount lidarMount = new LidarMount(lidarSensorDescription);
         joint.addLidarMount(lidarMount);

         //TODO: Should we really call addSensor here?
         // Instead, perhaps, there should be a better way to get the sensors from a robot...
         joint.addSensor(lidarMount);

         lidarNameMap.put(lidarMount.getName(), lidarMount);
      }
   }

   private void addCameraMounts(JointDescription jointDescription, Joint joint)
   {
      List<CameraSensorDescription> cameraSensorDescriptions = jointDescription.getCameraSensors();
      for (CameraSensorDescription cameraSensorDescription : cameraSensorDescriptions)
      {
         CameraMount cameraMount = new CameraMount(cameraSensorDescription.getName(),
                                                   cameraSensorDescription.getTransformToJoint(),
                                                   cameraSensorDescription.getFieldOfView(),
                                                   cameraSensorDescription.getClipNear(),
                                                   cameraSensorDescription.getClipFar(),
                                                   this);
         cameraMount.setImageWidth(cameraSensorDescription.getImageWidth());
         cameraMount.setImageHeight(cameraSensorDescription.getImageHeight());

         joint.addCameraMount(cameraMount);

         cameraNameMap.put(cameraMount.getName(), cameraMount);
      }
   }

   private void addIMUMounts(JointDescription jointDescription, Joint joint)
   {
      List<IMUSensorDescription> imuSensorDescriptions = jointDescription.getIMUSensors();
      for (IMUSensorDescription imuSensorDescription : imuSensorDescriptions)
      {
         IMUMount imuMount = new IMUMount(imuSensorDescription.getName(), imuSensorDescription.getTransformToJoint(), this);
         joint.addIMUMount(imuMount);

         imuNameMap.put(imuMount.getName(), imuMount);
      }
   }

   private void addJointWrenchSensors(JointDescription jointDescription, Joint joint)
   {
      List<JointWrenchSensorDescription> jointWrenchSensorDescriptions = jointDescription.getWrenchSensors();
      for (JointWrenchSensorDescription jointWrenchSensorDescription : jointWrenchSensorDescriptions)
      {
         JointWrenchSensor jointWrenchSensor = new JointWrenchSensor(jointWrenchSensorDescription.getName(),
                                                                     jointWrenchSensorDescription.getOffsetFromJoint(),
                                                                     this);
         joint.addJointWrenchSensor(jointWrenchSensor);

         wrenchSensorNameMap.put(jointWrenchSensor.getName(), jointWrenchSensor);
      }
   }

   private void addForceSensors(JointDescription jointDescription, Joint joint)
   {
      List<ForceSensorDescription> forceSensorDescriptions = jointDescription.getForceSensors();

      for (ForceSensorDescription forceSensorDescription : forceSensorDescriptions)
      {
         WrenchCalculatorInterface wrenchCalculator;

         if (forceSensorDescription.useGroundContactPoints())
         {
            //               System.out.println("SDFRobot: Adding old-school force sensor to: " + joint.getName());
            ArrayList<GroundContactPoint> groundContactPoints = new ArrayList<>();
            //TODO: Not sure if you want all of the ground contact points from here down, or just the ones attached to this joint.
            joint.recursiveGetAllGroundContactPoints(groundContactPoints);

            wrenchCalculator = new GroundContactPointBasedWrenchCalculator(forceSensorDescription.getName(),
                                                                           groundContactPoints,
                                                                           joint,
                                                                           forceSensorDescription.getTransformToJoint(),
                                                                           yoRegistry);
         }
         else
         {
            //               System.out.println("SDFRobot: Adding force sensor to: " + joint.getName());

            Vector3D offsetToPack = new Vector3D();
            offsetToPack.set(forceSensorDescription.getTransformToJoint().getTranslation());
            JointWrenchSensor jointWrenchSensor = new JointWrenchSensor(forceSensorDescription.getName(), offsetToPack, this);
            joint.addJointWrenchSensor(jointWrenchSensor);

            wrenchCalculator = new FeatherStoneJointBasedWrenchCalculator(forceSensorDescription.getName(), joint);
         }

         joint.addForceSensor(wrenchCalculator);
      }
   }

   private void addGroundContactPoints(JointDescription jointDescription, Joint joint)
   {
      List<GroundContactPointDescription> groundContactPointDescriptions = jointDescription.getGroundContactPoints();

      for (GroundContactPointDescription groundContactPointDescription : groundContactPointDescriptions)
      {
         GroundContactPoint groundContactPoint = new GroundContactPoint(groundContactPointDescription.getName(),
                                                                        groundContactPointDescription.getOffsetFromJoint(),
                                                                        this);
         joint.addGroundContactPoint(groundContactPointDescription.getGroupIdentifier(), groundContactPoint);

         if (!jointToGroundContactPointsMap.containsKey(joint))
         {
            jointToGroundContactPointsMap.put(joint, new ArrayList<GroundContactPoint>());
         }
         jointToGroundContactPointsMap.get(joint).add(groundContactPoint);
      }
   }

   private void addExternalForcePoints(JointDescription jointDescription, Joint joint)
   {
      List<ExternalForcePointDescription> ExternalForcePointDescriptions = jointDescription.getExternalForcePoints();

      for (ExternalForcePointDescription ExternalForcePointDescription : ExternalForcePointDescriptions)
      {
         ExternalForcePoint ExternalForcePoint = new ExternalForcePoint(ExternalForcePointDescription.getName(),
                                                                        ExternalForcePointDescription.getOffsetFromJoint(),
                                                                        this);
         joint.addExternalForcePoint(ExternalForcePoint);
      }
   }

   private void addKinematicPoints(JointDescription jointDescription, Joint joint)
   {
      List<KinematicPointDescription> KinematicPointDescriptions = jointDescription.getKinematicPoints();

      for (KinematicPointDescription KinematicPointDescription : KinematicPointDescriptions)
      {
         KinematicPoint KinematicPoint = new KinematicPoint(KinematicPointDescription.getName(), KinematicPointDescription.getOffsetFromJoint(), this);
         joint.addKinematicPoint(KinematicPoint);
      }
   }

   private Joint createSingleJoint(JointDescription jointDescription, boolean enableDamping, boolean enableJointTorqueAndVelocityLimits)
   {
      Joint joint;

      if (jointDescription instanceof FloatingJointDescription)
      {
         FloatingJointDescription floatingJointDescription = (FloatingJointDescription) jointDescription;

         Vector3D offset = new Vector3D();
         floatingJointDescription.getOffsetFromParentJoint(offset);

         joint = new FloatingJoint(jointDescription.getName(), floatingJointDescription.getJointVariableName(), offset, this, true);
      }

      else if (jointDescription instanceof FloatingPlanarJointDescription)
      {
         FloatingPlanarJointDescription floatingPlanarJointDescription = (FloatingPlanarJointDescription) jointDescription;

         joint = new FloatingPlanarJoint(jointDescription.getName(), this, floatingPlanarJointDescription.getPlane());
      }

      else if (jointDescription instanceof BallAndSocketJointDescription)
      {
         BallAndSocketJointDescription sphericalJointDescription = (BallAndSocketJointDescription) jointDescription;

         Vector3D offset = new Vector3D();
         sphericalJointDescription.getOffsetFromParentJoint(offset);

         joint = new BallAndSocketJoint(jointDescription.getName(), offset, this, true);
      }

      else if (jointDescription instanceof PinJointDescription)
      {
         PinJointDescription pinJointDescription = (PinJointDescription) jointDescription;
         Vector3D offset = new Vector3D();
         pinJointDescription.getOffsetFromParentJoint(offset);

         if (jointDescription.isDynamic())
         {
            Vector3D jointAxis = new Vector3D();
            pinJointDescription.getJointAxis(jointAxis);
            joint = new PinJoint(jointDescription.getName(), offset, this, jointAxis);

            PinJoint pinJoint = (PinJoint) joint;

            if (pinJointDescription.containsLimitStops())
            {
               double[] limitStopParameters = pinJointDescription.getLimitStopParameters();

               double qMin = limitStopParameters[0];
               double qMax = limitStopParameters[1];
               double kLimit = limitStopParameters[2];
               double bLimit = limitStopParameters[3];

               pinJoint.setLimitStops(qMin, qMax, kLimit, bLimit);
            }

            if (enableDamping)
            {
               pinJoint.setDamping(pinJointDescription.getDamping());
               pinJoint.setStiction(pinJointDescription.getStiction());
            }
            if (enableJointTorqueAndVelocityLimits)
            {
               pinJoint.setVelocityLimits(pinJointDescription.getVelocityLimit(), pinJointDescription.getVelocityDamping());
               pinJoint.setTorqueLimits(pinJointDescription.getEffortLimit());
            }
         }
         else
         {
            Vector3D jointAxis = new Vector3D();
            pinJointDescription.getJointAxis(jointAxis);
            joint = new DummyOneDegreeOfFreedomJoint(jointDescription.getName(), offset, this, jointAxis);
         }
      }
      else if (jointDescription instanceof SliderJointDescription)
      {
         SliderJointDescription sliderJointDescription = (SliderJointDescription) jointDescription;
         Vector3D offset = new Vector3D();
         sliderJointDescription.getOffsetFromParentJoint(offset);

         Vector3D jointAxis = new Vector3D();
         sliderJointDescription.getJointAxis(jointAxis);
         joint = new SliderJoint(jointDescription.getName(), offset, this, jointAxis);

         SliderJoint sliderJoint = (SliderJoint) joint;

         if (sliderJointDescription.containsLimitStops())
         {
            double[] limitStopParameters = sliderJointDescription.getLimitStopParameters();

            double qMin = limitStopParameters[0];
            double qMax = limitStopParameters[1];
            double kLimit = limitStopParameters[2];
            double bLimit = limitStopParameters[3];

            sliderJoint.setLimitStops(qMin, qMax, kLimit, bLimit);
         }

         if (enableDamping)
         {
            sliderJoint.setDamping(sliderJointDescription.getDamping());
            sliderJoint.setStiction(sliderJointDescription.getStiction());
         }

      }

      else
      {
         throw new RuntimeException("Don't support that joint type yet. Please implement it! Type = " + jointDescription.getClass());
      }

      if (!jointDescription.isDynamic())
      {
         joint.setDynamic(false);
      }

      LinkDescription linkDescription = jointDescription.getLink();

      if (linkDescription == null)
      {
         throw new RuntimeException("LinkDescription is null for joint " + jointDescription.getName());
      }
      Link link = createLink(linkDescription);
      joint.setLink(link);
      return joint;
   }

   private Link createLink(LinkDescription linkDescription)
   {
      Link link = new Link(linkDescription.getName());

      link.setMass(linkDescription.getMass());
      link.setComOffset(linkDescription.getCenterOfMassOffset());
      link.setMomentOfInertia(linkDescription.getMomentOfInertia());

      LinkGraphicsDescription linkGraphics = linkDescription.getLinkGraphics();
      link.setLinkGraphics(linkGraphics);

      List<CollisionMeshDescription> collisonMeshDescriptions = linkDescription.getCollisionMeshes();

      for (int i = 0; i < collisonMeshDescriptions.size(); i++)
      {
         link.addCollisionMesh(collisonMeshDescriptions.get(i));
      }

      return link;
   }

   private void constructRobotFromDefinition(RobotDefinition definition, boolean enableDamping, boolean enableJointTorqueAndVelocityLimits)
   {
      ClassLoader resourceClassLoader = definition.getResourceClassLoader();
      List<JointDefinition> rootJointDefinitions = definition.getRootJointDefinitions();

      for (JointDefinition rootJointDefinition : rootJointDefinitions)
      {
         Joint rootJoint = constructJointRecursively(rootJointDefinition,
                                                     definition.getNameOfJointsToIgnore(),
                                                     enableDamping,
                                                     enableJointTorqueAndVelocityLimits,
                                                     resourceClassLoader);
         addRootJoint(rootJoint);
      }

      for (JointDefinition rootJointDefinition : rootJointDefinitions)
      {
         addLoopClosureConstraintsRecursively(rootJointDefinition);
      }
   }

   private Joint constructJointRecursively(JointDefinition jointDefinition,
                                           Collection<String> jointsToIgnore,
                                           boolean enableDamping,
                                           boolean enableJointTorqueAndVelocityLimits,
                                           ClassLoader resourceClassLoader)
   {
      Joint joint = createSingleJoint(jointDefinition, jointsToIgnore, enableDamping, enableJointTorqueAndVelocityLimits, resourceClassLoader);

      addGroundContactPoints(jointDefinition, joint);
      addExternalForcePoints(jointDefinition, joint);
      addKinematicPoints(jointDefinition, joint);

      addLidarMounts(jointDefinition, joint);
      addCameraMounts(jointDefinition, joint);
      addIMUMounts(jointDefinition, joint);
      addForceSensors(jointDefinition, joint);

      // Iterate over the children
      List<JointDefinition> childrenJoints = jointDefinition.getSuccessor().getChildrenJoints();
      for (JointDefinition childJointDefinition : childrenJoints)
      {
         if (childJointDefinition.isLoopClosure())
            continue; // Loop closures are added afterward.
         Joint childJoint = constructJointRecursively(childJointDefinition,
                                                      jointsToIgnore,
                                                      enableDamping,
                                                      enableJointTorqueAndVelocityLimits,
                                                      resourceClassLoader);
         joint.addJoint(childJoint);
      }

      jointNameMap.put(joint.getName(), joint);

      if (joint instanceof OneDegreeOfFreedomJoint)
      {
         oneDegreeOfFreedomJoints.put(joint.getName(), (OneDegreeOfFreedomJoint) joint);
      }

      return joint;
   }

   private Joint createSingleJoint(JointDefinition jointDefinition,
                                   Collection<String> jointsToIgnore,
                                   boolean enableDamping,
                                   boolean enableJointTorqueAndVelocityLimits,
                                   ClassLoader resourceClassLoader)
   {
      Joint joint;

      if (jointDefinition instanceof SixDoFJointDefinition)
      {
         SixDoFJointDefinition sixDoFJointDefinition = (SixDoFJointDefinition) jointDefinition;

         Vector3D offset = new Vector3D(sixDoFJointDefinition.getTransformToParent().getTranslation());
         joint = new FloatingJoint(jointDefinition.getName(), sixDoFJointDefinition.getVariableName(), offset, this, true);
         joint.getOffsetTransform3D().set(jointDefinition.getTransformToParent());
      }
      else if (jointDefinition instanceof PlanarJointDefinition)
      {
         PlanarJointDefinition planarJointDefinition = (PlanarJointDefinition) jointDefinition;

         joint = new FloatingPlanarJoint(planarJointDefinition.getName(), this, Plane.XZ);
         joint.getOffsetTransform3D().set(jointDefinition.getTransformToParent());
      }
      else if (jointDefinition instanceof RevoluteJointDefinition)
      {
         RevoluteJointDefinition revoluteJointDefinition = (RevoluteJointDefinition) jointDefinition;
         Vector3D offset = new Vector3D(revoluteJointDefinition.getTransformToParent().getTranslation());

         if (jointsToIgnore.contains(revoluteJointDefinition.getName()))
         {
            joint = new DummyOneDegreeOfFreedomJoint(jointDefinition.getName(), offset, this, revoluteJointDefinition.getAxis());
         }
         else
         {
            joint = new PinJoint(jointDefinition.getName(), offset, this, revoluteJointDefinition.getAxis());

            PinJoint pinJoint = (PinJoint) joint;

            if (!Double.isNaN(revoluteJointDefinition.getPositionLowerLimit()) && !Double.isNaN(revoluteJointDefinition.getPositionUpperLimit()))
            {
               double qMin = revoluteJointDefinition.getPositionLowerLimit();
               double qMax = revoluteJointDefinition.getPositionUpperLimit();
               double kLimit = revoluteJointDefinition.getKpSoftLimitStop();
               double bLimit = revoluteJointDefinition.getKdSoftLimitStop();
               if (kLimit < 0.0 || Double.isNaN(kLimit))
                  kLimit = 0.0;
               if (bLimit < 0.0 || Double.isNaN(bLimit))
                  bLimit = 0.0;

               pinJoint.setLimitStops(qMin, qMax, kLimit, bLimit);
            }

            if (enableDamping)
            {
               if (revoluteJointDefinition.getDamping() >= 0.0)
                  pinJoint.setDamping(revoluteJointDefinition.getDamping());
               if (revoluteJointDefinition.getStiction() >= 0.0)
                  pinJoint.setStiction(revoluteJointDefinition.getStiction());
            }
            if (enableJointTorqueAndVelocityLimits)
            {
               pinJoint.setVelocityLimits(revoluteJointDefinition.getVelocityUpperLimit(), revoluteJointDefinition.getDampingVelocitySoftLimit());
               pinJoint.setTorqueLimits(revoluteJointDefinition.getEffortUpperLimit());
            }
         }
         joint.getOffsetTransform3D().set(jointDefinition.getTransformToParent());
      }
      else if (jointDefinition instanceof PrismaticJointDefinition)
      {
         PrismaticJointDefinition prismaticJointDefinition = (PrismaticJointDefinition) jointDefinition;
         Vector3D offset = new Vector3D(prismaticJointDefinition.getTransformToParent().getTranslation());

         joint = new SliderJoint(jointDefinition.getName(), offset, this, prismaticJointDefinition.getAxis());

         SliderJoint sliderJoint = (SliderJoint) joint;

         if (!Double.isNaN(prismaticJointDefinition.getPositionLowerLimit()) && !Double.isNaN(prismaticJointDefinition.getPositionUpperLimit()))
         {
            double qMin = prismaticJointDefinition.getPositionLowerLimit();
            double qMax = prismaticJointDefinition.getPositionUpperLimit();
            double kLimit = prismaticJointDefinition.getKpSoftLimitStop();
            double bLimit = prismaticJointDefinition.getKdSoftLimitStop();
            if (kLimit < 0.0 || Double.isNaN(kLimit))
               kLimit = 0.0;
            if (bLimit < 0.0 || Double.isNaN(bLimit))
               bLimit = 0.0;

            sliderJoint.setLimitStops(qMin, qMax, kLimit, bLimit);
         }

         if (enableDamping)
         {
            if (prismaticJointDefinition.getDamping() >= 0.0)
               sliderJoint.setDamping(prismaticJointDefinition.getDamping());
            if (prismaticJointDefinition.getStiction() >= 0.0)
               sliderJoint.setStiction(prismaticJointDefinition.getStiction());
         }
         joint.getOffsetTransform3D().set(jointDefinition.getTransformToParent());
      }
      else if (jointDefinition instanceof FixedJointDefinition)
      {
         joint = new RigidJoint(jointDefinition.getName(), jointDefinition.getTransformToParent().getTranslation(), this);
         joint.getOffsetTransform3D().set(jointDefinition.getTransformToParent());
      }
      else
      {
         throw new RuntimeException("Don't support that joint type yet. Please implement it! Type = " + jointDefinition.getClass());
      }

      if (jointsToIgnore.contains(joint.getName()))
      {
         joint.setDynamic(false);
      }

      RigidBodyDefinition rigidBodyDefinition = jointDefinition.getSuccessor();

      if (rigidBodyDefinition == null)
      {
         throw new RuntimeException("RigidBodyDefinition is null for joint " + jointDefinition.getName());
      }
      Link link = createLink(rigidBodyDefinition, resourceClassLoader);
      joint.setLink(link);
      return joint;
   }

   private Link createLink(RigidBodyDefinition rigidBodyDefinition, ClassLoader resourceClassLoader)
   {
      Link link = new Link(rigidBodyDefinition.getName());

      link.setMass(rigidBodyDefinition.getMass());
      link.setComOffset(rigidBodyDefinition.getCenterOfMassOffset());
      link.setMomentOfInertia(rigidBodyDefinition.getMomentOfInertia());
      link.setLinkGraphics(VisualsConversionTools.toGraphics3DObject(rigidBodyDefinition.getVisualDefinitions()));

      return link;
   }

   private void addGroundContactPoints(JointDefinition jointDefinition, Joint joint)
   {
      List<GroundContactPointDefinition> groundContactPointDefinitions = jointDefinition.getGroundContactPointDefinitions();

      for (GroundContactPointDefinition groundContactPointDefinition : groundContactPointDefinitions)
      {
         GroundContactPoint groundContactPoint = new GroundContactPoint(groundContactPointDefinition.getName(),
                                                                        groundContactPointDefinition.getTransformToParent().getTranslation(),
                                                                        this);
         joint.addGroundContactPoint(groundContactPointDefinition.getGroupIdentifier(), groundContactPoint);

         if (!jointToGroundContactPointsMap.containsKey(joint))
         {
            jointToGroundContactPointsMap.put(joint, new ArrayList<GroundContactPoint>());
         }
         jointToGroundContactPointsMap.get(joint).add(groundContactPoint);
      }
   }

   private void addExternalForcePoints(JointDefinition jointDefinition, Joint joint)
   {
      List<ExternalWrenchPointDefinition> externalWrenchPointDescriptions = jointDefinition.getExternalWrenchPointDefinitions();

      for (ExternalWrenchPointDefinition externalWrenchPointDescription : externalWrenchPointDescriptions)
      {
         ExternalForcePoint ExternalForcePoint = new ExternalForcePoint(externalWrenchPointDescription.getName(),
                                                                        externalWrenchPointDescription.getTransformToParent().getTranslation(),
                                                                        this);
         joint.addExternalForcePoint(ExternalForcePoint);
      }
   }

   private void addKinematicPoints(JointDefinition jointDefinition, Joint joint)
   {
      List<KinematicPointDefinition> kinematicPointDefinitions = jointDefinition.getKinematicPointDefinitions();

      for (KinematicPointDefinition kinematicPointDefinition : kinematicPointDefinitions)
      {
         KinematicPoint KinematicPoint = new KinematicPoint(kinematicPointDefinition.getName(),
                                                            kinematicPointDefinition.getTransformToParent().getTranslation(),
                                                            this);
         joint.addKinematicPoint(KinematicPoint);
      }
   }

   private void addLidarMounts(JointDefinition jointDefinition, Joint joint)
   {
      List<LidarSensorDefinition> lidarSensorDefinitions = jointDefinition.getSensorDefinitions(LidarSensorDefinition.class);

      for (LidarSensorDefinition lidarSensorDefinition : lidarSensorDefinitions)
      {
         LidarMount lidarMount = new LidarMount(toLidarSensorDescription(lidarSensorDefinition));
         joint.addLidarMount(lidarMount);

         //TODO: Should we really call addSensor here?
         // Instead, perhaps, there should be a better way to get the sensors from a robot...
         joint.addSensor(lidarMount);

         lidarNameMap.put(lidarMount.getName(), lidarMount);
      }
   }

   private void addCameraMounts(JointDefinition jointDefinition, Joint joint)
   {
      List<CameraSensorDefinition> cameraSensorDefinitions = jointDefinition.getSensorDefinitions(CameraSensorDefinition.class);
      for (CameraSensorDefinition cameraSensorDefinition : cameraSensorDefinitions)
      {
         CameraMount cameraMount = new CameraMount(cameraSensorDefinition.getName(),
                                                   cameraSensorDefinition.getTransformToJoint(),
                                                   cameraSensorDefinition.getFieldOfView(),
                                                   cameraSensorDefinition.getClipNear(),
                                                   cameraSensorDefinition.getClipFar(),
                                                   this);
         cameraMount.setImageWidth(cameraSensorDefinition.getImageWidth());
         cameraMount.setImageHeight(cameraSensorDefinition.getImageHeight());

         joint.addCameraMount(cameraMount);

         cameraNameMap.put(cameraMount.getName(), cameraMount);
      }
   }

   private void addIMUMounts(JointDefinition jointDefinition, Joint joint)
   {
      List<IMUSensorDefinition> imuSensorDefinitions = jointDefinition.getSensorDefinitions(IMUSensorDefinition.class);
      for (IMUSensorDefinition imuSensorDefinition : imuSensorDefinitions)
      {
         IMUMount imuMount = new IMUMount(imuSensorDefinition.getName(), new RigidBodyTransform(imuSensorDefinition.getTransformToJoint()), this);
         joint.addIMUMount(imuMount);

         imuNameMap.put(imuMount.getName(), imuMount);
      }
   }

   private void addForceSensors(JointDefinition jointDefinition, Joint joint)
   {
      List<WrenchSensorDefinition> wrenchSensorDefinitions = jointDefinition.getSensorDefinitions(WrenchSensorDefinition.class);

      for (WrenchSensorDefinition wrenchSensorDefinition : wrenchSensorDefinitions)
      {
         WrenchCalculatorInterface wrenchCalculator;

         //               System.out.println("SDFRobot: Adding old-school force sensor to: " + joint.getName());
         ArrayList<GroundContactPoint> groundContactPoints = new ArrayList<>();
         //TODO: Not sure if you want all of the ground contact points from here down, or just the ones attached to this joint.
         joint.recursiveGetAllGroundContactPoints(groundContactPoints);

         wrenchCalculator = new GroundContactPointBasedWrenchCalculator(wrenchSensorDefinition.getName(),
                                                                        groundContactPoints,
                                                                        joint,
                                                                        new RigidBodyTransform(wrenchSensorDefinition.getTransformToJoint()),
                                                                        yoRegistry);

         joint.addForceSensor(wrenchCalculator);
      }
   }

   public static LidarSensorDescription toLidarSensorDescription(LidarSensorDefinition source)
   {
      LidarSensorDescription output = new LidarSensorDescription(source.getName(), source.getTransformToJoint());
      output.setSweepYawMin(source.getSweepYawMin());
      output.setSweepYawMax(source.getSweepYawMax());
      output.setHeightPitchMin(source.getHeightPitchMin());
      output.setHeightPitchMax(source.getHeightPitchMax());
      output.setMinRange(source.getMinRange());
      output.setMaxRange(source.getMaxRange());
      output.setPointsPerSweep(source.getPointsPerSweep());
      output.setScanHeight(source.getScanHeight());
      return output;
   }

   private void addLoopClosureConstraintsRecursively(JointDefinition jointDefinition)
   {
      Joint joint = jointNameMap.get(jointDefinition.getName());

      List<JointDefinition> childJointDefinitions = jointDefinition.getSuccessor().getChildrenJoints();

      for (JointDefinition childJointDefinition : childJointDefinitions)
      {
         if (!childJointDefinition.isLoopClosure())
            continue;

         LoopClosureDefinition loopClosureDefinition = childJointDefinition.getLoopClosureDefinition();

         String name = childJointDefinition.getName();
         Tuple3DReadOnly offsetFromParentJoint = childJointDefinition.getTransformToParent().getTranslation();
         Tuple3DReadOnly offsetFromLinkParentJoint = loopClosureDefinition.getTransformToSuccessorParent().getTranslation();
         Matrix3DReadOnly constraintForceSubSpace = LoopClosureDefinition.jointForceSubSpace(jointDefinition);
         Matrix3DReadOnly constraintMomentSubSpace = LoopClosureDefinition.jointMomentSubSpace(jointDefinition);

         if (constraintForceSubSpace == null || constraintMomentSubSpace == null)
            throw new UnsupportedOperationException("Loop closure not supported for " + jointDefinition);

         LoopClosureSoftConstraint constraint = new LoopClosureSoftConstraint(name,
                                                                              offsetFromParentJoint,
                                                                              offsetFromLinkParentJoint,
                                                                              this,
                                                                              constraintForceSubSpace,
                                                                              constraintMomentSubSpace);
         constraint.setGains(loopClosureDefinition.getKpSoftConstraint(), loopClosureDefinition.getKdSoftConstraint());

         joint.addLoopClosureConstraint(constraint);
         Link link = getLink(childJointDefinition.getSuccessor().getName());
         Objects.requireNonNull(link, "Could not find link: " + childJointDefinition.getSuccessor().getName());
         constraint.setLink(link);
      }

      for (JointDefinition childJointDefinition : childJointDefinitions)
      {
         addLoopClosureConstraintsRecursively(childJointDefinition);
      }
   }
}
