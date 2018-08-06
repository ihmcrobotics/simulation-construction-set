---
title: Create a New Class FallingBrickRobot
---

### Create a New Separate Class Called FallingBrickRobot
   Fill the class with the following:
   
   These are simply the initial variables that are set separately so that they can be more easily changed later on.

```java
package us.ihmc.exampleSimulations.fallingBrick;
import javax.vecmath.Vector3d;
import us.ihmc.jMonkeyEngineToolkit.GroundProfile3D;
import us.ihmc.robotics.robotDescription.LinkGraphicsDescription;
import us.ihmc.graphicsDescription.appearance.AppearanceDefinition;
import us.ihmc.graphicsDescription.appearance.YoAppearance;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.simulationconstructionset.FloatingJoint;
import us.ihmc.simulationconstructionset.GroundContactModel;
import us.ihmc.simulationconstructionset.GroundContactPoint;
import us.ihmc.simulationconstructionset.Joint;
import us.ihmc.simulationconstructionset.Link;
import us.ihmc.simulationconstructionset.Robot;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.robotController.RobotController;
import us.ihmc.simulationconstructionset.util.LinearGroundContactModel;
import us.ihmc.simulationconstructionset.util.ground.WavyGroundProfile;
public class FallingBrickRobot extends Robot implements RobotController
{
   private static final long serialVersionUID = 773713164696806099L;
   
   private static final double BASE_H = 0.1, BASE_W = 0.2, BASE_L = 0.3;
   private static final double B1 = BASE_H / 2.0;
   private static final double M1 = 1.7;
   private static final double Ixx1 = 0.1, Iyy1 = 0.5, Izz1 = 0.9;
   private static final double G = 9.81;
   private final YoVariableRegistry registry = new YoVariableRegistry("FallingBrickController");
   // position, velocity, and acceleration variables
   DoubleYoVariable q_x, q_y, q_z, qd_x, qd_y, qd_z, qdd_x, qdd_y, qdd_z;
   DoubleYoVariable q_qs, q_qx, q_qy, q_qz, qd_wx, qd_wy, qd_wz, qdd_wx, qdd_wy, qdd_wz;
   DoubleYoVariable energy, q_qlength, theta_x;
   DoubleYoVariable qdd2_wx, qdd2_wy, qdd2_wz;
   Joint floatingJoint;
}
```