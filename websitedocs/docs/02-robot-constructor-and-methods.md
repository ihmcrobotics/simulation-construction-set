---
title: Robot Constructor and Methods
---

Table 5 contains the constructors and methods for the class Robot.
To create a Robot you first call its constructor with its name, then create a tree of joints, and then set the root joint of that tree using Robot.addRootJoint(Joint root). 
You may add multiple root joints using addRootJoint, if your Robot is comprised of several independent trees of Joints. 
If the robot has a control system or ground contact points, you set the controller using setController and set the GroundContactModel using setGroundContactModel. 
To change the value of a variable in your robot, use setVariable. To extract a YoVariable from the robot, use getVar() with the name of the variable.
 
 
 
Creates a new robot.
```java
Robot(String name)
```
 
Adds a root joint for this robot.
```java
void addRootJoint(Joint root)
```
  
Sets gravity for this robot. Default is (0.0, 0.0, -9.81).
```java
void setGravity(double gX, double gY, double gZ)
public void setGravity(double gZ) 
```
  
Sets the RobotController for this robot.
```java
void setController(RobotController c)
```

Sets the GroundContactModel for this robot.
```java
void setGroundContactModel(GroundContactModel)
```

Returns the ground contact points associated with this robot. They will be stored in an ArrayList, which is a Java utility class in (java.util.ArrayList)
```java
ArrayList getGroundContactPoints()
```

Gets or sets the variable with the given name.
```java
YoVariable getVar(String name)
void setVariable(String name, double val)
```

Adds a static Link with no physics, or a static Java3D BranchGroup. For creating the surrounding static world graphics.
```java
void addStaticLink(Link staticLink) 
void addStaticBranchGroup(BranchGroup staticBranchGroup)
```