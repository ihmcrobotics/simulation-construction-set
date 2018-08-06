---
title: Introduction to special points
---


Sometimes you wish to monitor the Cartesian position and velocity of a point on your robot, apply an external force to that point, or model ground contact at that point.

* The first task can be accomplished using a KinematicPoint. 
* The second can be accomplished using an ExternalForcePoint which extends KinematicPoint. 
* The third can be accomplished using a GroundContactPoint which extends ExternalForcePoint. 

The Creation of a point requires a **name**, an **offset**, and a **robot**. 

The **offset** is the vector from the joint that the point will be attached to using the *addKinematicPoint*, *addExternalForcePoint*, or *addGroundContactPoint* methods of Joint. 
The point will move with the Joint in the same way that a Link will move with the Joint. 


When a **KinematicPoint** is created, the YoVariables "name_x", "name_y", "name_z", "name_dx", "name_dy", "name_dz" are automatically created.
These are the position and velocity (in world coordinates) variables for the point. These variables get updated whenever the robot is updated. 


The class **ExternalForcePoint** is an extension of KinematicPoint. When an ExternalForcePoint is created, the YoVariables "name_fx", "name_fy", "name_fz" are automatically created. 
These are the forces acting on that point (in world coordinates). 
These variables can be set by the user to simulated a force acting on the robot at that point, or can be set by a ground contact model, or other object if the ExternalForcePoint is being used to model a contact or other event. 


The class **GroundContactPoint** is an extension of ExternalForcePoint. 
GroundContactPoints also have the YoVariables tdx, tdy, tdz which is the location where the GroundContactPoint first contacts the ground and fs, which is a footswitch. 
If fs.val equals 1.0, then the GroundContactPoint is contacting the ground, if it equals 0.0, the point is not contacting the ground. To turn the GroundContactPoint off, set fs.val equal to -1.0. 

Given a KinematicPoint, ExternalForcePoint, or GroundContactPoint, you can extract the automatically created variables. 

For example the following code will create an ExternalForcePoint, attach it to a Joint, and apply a force of 1.0 in the Z direction:

```java
point1 = ExternalForcePoint("efp1", new Vector3d(), rob);
previousJoint.addExternalForcePoint(point1);
YoVariable efp1_fz = point1.fz;
efp1_fz.val = 1.0;
```

### Related Example:

[Example of special points in use](https://ihmcrobotics.github.io/simulation-construction-set/docs/01-implementing-closed-chain-mechanisms.html)