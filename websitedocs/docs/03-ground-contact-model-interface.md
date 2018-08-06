---
title: GroundContactModel interface
---

The forces applied to a GroundContactPoint are determined by the GroundContactModel which is set for the Robot.

GroundContactModel is an interface with the required methods doGroundContact, setGroundProfile, and getGroundProfile.
 The code for implementing your ground contact model should be placed in doGroundContact. There are also a couple GroundContactModels that are in the com.yobotics.simulationconstructionset.utils package. 
A GroundProfile defines the contour of your terrain. 

Method for setting the ground contact forces given their positions, velocities, and the ground profile.
```java
public abstract void doGroundContact();
```

Sets and returns the GroundProfile.
```java
public abstract void setGroundProfile(GroundProfile profile); 
public abstract GroundProfile getGroundProfile();
```

### Related Example: 

[Example of GroundContactModel in Use](https://ihmcrobotics.github.io/simulation-construction-set/docs/01a-create-new-package.html)