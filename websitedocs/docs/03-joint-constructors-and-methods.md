---
title: Joint Constructor and methods
---


Table 6 lists the Joint constructors and the methods which are specific to the type of joint. The first two Joints, FloatingJoint and PlanarFloatingJoint, can not be children of any other joint and cannot apply torques.
They are used for free bodies, such as the body of a walking robot. If the body is unconstrained (has 6 degrees of freedom – 3 translations and 3 rotations), then use a FloatingJoint. 
If the body is confined to a plane (2 translations and 1 rotation perpendicular to the 2 translations), then use a FloatingPlanarJoint. 
The other types of joints are either PinJoint, which allows rotation, or SliderJoint, which allows translation, or some combination of PinJoints and SliderJoints. 
A CylinderJoint is a PinJoint and a SliderJoint, with rotation and translation both about the same axis. A UniversalJoint is 2 perpendicular PinJoints. A GimbalJoint consists of 3 perpendicular PinJoints. 
In creating a Joint, you must specify the offset vector from the previous joint. This vector is defined when the rotations and translations of the previous joint are set to zero. 
For example, if a humanoid robot is created such that its arms hang straight down when the shoulders joint angles are zero, then the vector from the shoulder to the elbow will be (0.0,0.0,-UPPER_ARM_LENGTH). 
In creating a Joint, you must also specify the axes of rotation as one of Joint.X, Joint.Y, or Joint.Z. In the case of PinJoints and SliderJoints, you may also specify an axis of rotation by providing a Vector3d that defines the axis. 
Most of the joints have the capability for setting damping, limit stops, and for setting their initial state. Damping is implemented as a linear viscous friction (torque is proportional to velocity).
Limit stops are implemented as virtual spring-damper systems, with the given spring and damping constants. 

Creates a 6 degree of freedom floating joint. This joint can only be the root joint for a robot. YoVariables will be automatically created and added to the robot. If no varName is given, these variables will be q_(x,y,z), qd_(x,y,z), qdd_(x,y,z), q_q(s,x,y,z), qd_w(x,y,z), qdd_w(x,y,z). q_q(s,x,y,z) is the rotation of the joint, expresses as a quaternion, while qd_w(x,y,z) is the rotational velocity of the joint.
```java
FloatingJoint(String jname, String varName, Vector3d offset, Robot rob) 
FloatingJoint(String jname, Vector3d offset, Robot rob)
FloatingPlanarJoint(String jname, Robot rob) 
```

Creates a 3 degree of freedom planar joint. This joint can only be the root joint for a robot. YoVariables q_(t1,t2,rot), qd_(t1,t2,rot), qdd_(t1,t2,rot) and tau_(t1,t2,rot) will be automatically created and added to the robot, where t1, t2, are replaced with x,y, or z; and rot is replaced with roll, pitch, or yaw depending if the axis of rotation is x,y,or z. The type is one of XY, XZ, or YZ which defines the plane of motion. If no type is selected, then XZ is the default.
```java
FloatingPlanarJoint(String jname, Robot rob, int type)
```

Constant parameters for joint type selection when creating a PlanarFloatingJoint.
```java
public static final int XY; 
public static final int YZ; 
public static final int XZ;
```

Creates a pin joint with variables q_, qd_, qdd_, tau_ with * replaced by the joint name. Rotation is about jaxis which can be Joint.X, Joint.Y, or Joint.Z. offset is the Vector3d from the previous joint.
```java
PinJoint(String jname, Vector3d offset, Robot rob, int jaxis)
```

Creates a pin joint as above, but with the Joint Axis pointing along the Vector3d jaxis.
```java
PinJoint(String jname, Vector3d offset, Robot rob, Vector3d jaxis)
```

Sets the position and velocity of this joint.
```java
void setInitialState(double q_init, double qd_init)
```

Adds viscous damping to the pin joint.
```java
void setDamping(double b_damp)
```

Adds limit stops to the pin joint. These limit stops are implemented as a spring-damper system with spring constant k_limit and damper constant b_limit.
```java
void setLimitStops(double q_min, double q_max, double k_limit, double b_limit)
```

Creates a slider joint with variables q_, qd_, qdd_, tau_ with * replaced by the joint name. Translation is about jaxis which can be Joint.X, Joint.Y, or Joint.Z. offset is the Vector3d from the previous joint.
```java
SliderJoint(String jname, Vector3d offset, Robot rob, int jaxis)
```

Creates a slider joint as above, but with the Joint Axis pointing along the Vector3d jaxis.
```java
SliderJoint(String jname, Vector3d offset, Robot rob, Vector3d jaxis)
```

Sets the position and velocity of this joint.
```java
void setInitialState(double q_init, double qd_init)
```

Adds viscous damping to the slider joint.
```java
void setDamping(double b_damp)
```

Adds limit stops to the slider joint. These limit stops are implemented as a spring-damper system with spring constant k_limit and damper constant b_limit.
```java
void setLimitStops(double q_min, double q_max, double k_limit, double b_limit)
```

Creates a cylinder joint, which is a pin joint plus a slider joint. Both rotation and translation are about the same axis, which can be Joint.X, Joint.Y, or Joint.Z.
```java
CylinderJoint(String rotName, String transName, Vector3d offset, Robot rob, int jaxis)
```

Sets the positions and velocities of this joint.
```java
void setInitialState(double q1_init, double qd1_init, double q2_init, double qd2_init)
```

Adds damping to the indicated axis of the CylinderJoint (1 = pin, 2 = slider).
```java
void setDamping(int axis, double b_damp)
```

Adds limit stops to the indicated axis of the CylinderJoint.
```java
void setLimitStops(int axis, double q_min, double q_max, double k_limit, double b_limit)
```

Creates a universal joint, which is two pin joints in a row. Rotations are about firstAxis and secondAxis, which can be Joint.X, Join.Y, or Joint.Z. The same axis cannot be used for both joints however.
```java
UniversalJoint(String jname1, String jname2, Vector3d offset, Robot rob, int firstAxis, int secondAxis)
```

Sets the positions and velocities of this joint.
```java
void setInitialState(double q1_init, double qd1_init, double q2_init, double qd2_init)
```

Adds damping to the indicated axis of the UniversalJoint (1 = firstAxis, 2 = secondAxis).
```java
void setDamping(int axis, double b_damp)
```

Adds limit stops to the indicated axis of the UniversalJoint.
```java
void setLimitStops(int axis, double q_min, double q_max, double k_limit, double b_limit)
```

Creates a gimbal joint, which is three pin joints in a row. The axis of rotation must all be perpendicular (one each of Joint.X, Joint.Y, and Joint.Z)
```java
GimbalJoint(String jname1, String jname2, String jname3, Vector3d offset, Robot rob, int firstAxis, int secondAxis, int thirdAxis)
```

Sets the positions and velocities of this joint.
```java
void setInitialState(double q1_init, double qd1_init, double q2_init, double qd2_init, double q3_init, double qd3_init)
```

Adds damping to the indicated axis of the GimbalJoint (1 = firstAxis, 2 = secondAxis, 3 = thirdAxis).
```java
void setDamping(int axis, double b_damp)
```

Adds limit stops to the indicated axis of the GimbalJoint.
```java
void setLimitStops(int axis, double q_min, double q_max, double k_limit, double b_limit)
```

### Related Example:

[Example of Joint Constructor](https://ihmcrobotics.github.io/simulation-construction-set/docs/01-create-new-package.html)
[Example of Joint Methods](https://ihmcrobotics.github.io/simulation-construction-set/docs/01-implementing-closed-chain-mechanisms.html)  