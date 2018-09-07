---
title: General Joint Methods
---

Table 7 lists the methods which all Joints share. Each joint has a corresponding link. 
The link must be created as described in the Link API description from above. To set the link to the joint you use the setLink(Link l) method. 
To add a child Joint to another Joint, use the addJoint(Joint nextJoint) method. 
KinematicPoint, ExternalForcePoint and GroundContactPoint create various points that can be added using the corresponding methods.
A KinematicPoint can be used to track the world coordinates and velocity of a point on the robot. ExternalForcePoint extends KinematicPoint and adds functionality for applying forces or impulses to the point. 
GroundContactPoint extends ExternalForcePoint and adds functionality for ground contact modeling. Each type of point is added to its parent Joint and moves with that joint. 


Sets the link for this joint.
```java
void setLink(Link l)
```

Adds a child joint to this joint.
```java
void addJoint (Joint nextJoint)
```

Adds a child kinematic point to this joint.
```java
void addKinematic (KinematicPoint point)
```

Adds a child external force point to this joint.
```java
void addExternalForcePoint (ExternalForcePoint point)
```

Adds a child ground contact point to this joint.
```java
void addGroundContactPoint (GroundContactPoint point)
```

Changes the offset Vector for this Joint. Should only be used to change a robot's shape (and not as a way to actuate a degree of freedom), since the use of this function does not obey the laws of physics.
```java
void changeOffsetVector(Vector3d newOffsetVector) 
void changeOffsetVector(double x, double y, double z)
```

Constants for specifying the joint axis in the joint constructors.
```java
public static final int Axis.X; 
public static final int Axis.Y; 
public static final int Axis.Z;
```