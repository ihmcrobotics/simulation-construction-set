---
title: Link constructor and methods
---

Link constructor and methods for setting the mass, center of mass offset, and moment of inertia. 
These methods affect the dynamics of the link. All of the others are for graphical purposes only. The center of mass offset is the vector from the joint to the center of mass. 
The moment of inertia is defined about the center of mass. Thus a moment of inertia of zero corresponds to a point mass.

Creating a Link and Setting its Properties

Creates a link with name lname.
```java 
Link(String lname) 
```

Sets the mass of the link.
```java
void setMass(double mass)
```

Sets the moment of inertia of the link about the center of mass.
```java
void setMomentOfInertia(double Ixx, double Iyy, double Izz)
```

Sets the center of mass offset of the link with respect to its corresponding joint.
```java
void setComOffset(double xOffset, double yOffset, double zOffset)
```

Sets the associated graphics properties object for this link. Graphics3DObject is an abstraction that LinkGraphicsDescription implements.
```java
Void setLinkGraphics(Graphics3DObject linkGraphics)
```

### Related Example:

[Example of how Link Constructors are used](https://ihmcrobotics.github.io/simulation-construction-set/docs/01-creating-links.html)