---
title: LinkGraphicsDescription constructor and methods
---

Part 2 lists the LinkGraphicsDescription constructor and methods for adding rotation and translation. These transformations accumulate until identity() is called,
in which the transformation for new shapes to be added is reset to the origin. Note that if you rotate and then translate, the translation will be along the new coordinate system defined by the rotation.
For example, **rotate(Math.PI/2.0, Axis.X); translate(0.0,1.0,0.0);** is equivalent to **translate(0.0,0.0,1.0); rotate(Math.PI/2.0, Axis.X);**

Creates an instance of LinkGraphicsDescription
```java
LinkGraphicsDescription()
```

Reset back to the joint origin.
```java
void identity()
```

Translate by (tx,ty,tz)
```java
void translate(double tx, double ty, double tz)
```


Rotate by rotAng about rotAxis. rotAxis can be Axis.X, Axis.Y, or Axis.Z
```java
void rotate(double rotAng, int rotAxis)
```

Constants for specifying axis of rotation for Link.rotate method.
```java
public static final int Axis.X; 
public static final int Axis.Y; 
public static final int Axis.Z;
```

### Related Example:  

[Example of how LinkGraphicsDescription is used](https://ihmcrobotics.github.io/simulation-construction-set/docs/01-creating-links.html)
