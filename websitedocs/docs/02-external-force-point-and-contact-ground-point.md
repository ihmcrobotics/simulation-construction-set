---
title: ExternalForcePoint and GroundContactPoint constructors and methods
---



Creates an external force point, automatically creating the variables name_(x,y,z,dx,dy,dz) and adding them to the given Robot. offset is the Vector3d from the parent joint.
```java
ExternalForcePoint(String name, Vector3d offset, Robot rob)
```

YoVariables for the cartesian position, and velocity of the kinematic point. These values are automatically computed by the simulation.
```java
public YoVariable x, y, z; 
public YoVariable dx, dy, dz;
```

Creates an external force point, which is an extension of a kinematic point. In addition to the kinematic point variables, name_(fx,fy,fz) are automatically generated and added to the Robot.
```java
ExternalForcePoint(String name, Vector3d offset, Robot rob)
```

YoVariables for the force on the external force point. Force is input by the user, or automatically computed from another object such as a GroundContacModel.
```java
public YoVariable fx, fy, fz;
```

Creates a ground contact point, which is an extension of an external force point. In addition to the external force point variables, name_(tdx,tdy,tdz,fs) are automatically generated and added to the Robot.
```java
GroundContactPoint(String name, Vector3d offset, Robot rob)
public YoVariable tdx, tdy, tdz; 
```

Touch-down position and footswitch. fs.val = 1.0 during foot contact and 0.0 otherwise. To turn the ground contact point off, set fs.val to -1.0.
```java
public YoVariable fs;
```


### Related Example:

[Example of ExternalForcePoint in use](https://ihmcrobotics.github.io/simulation-construction-set/docs/01-implementing-closed-chain-mechanisms.html)