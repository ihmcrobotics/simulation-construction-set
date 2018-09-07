---
title: Adding shapes
---

Table 3 lists the methods available for adding shapes. 
The first method addShape(Geometry, Appearance) is a lower-level interface for creating a shape using the given Geometry and Appearance.
Geometry and Appearance are classes defined in the Java3D API. This function is for advanced users who need to make custom shapes.
We recommend reading Java3D reference manuals and becoming familiar with the Java3D API before using this method. 
The other methods are easy to use interfaces for adding standard shapes such as cubes, spheres, cones, ellipses, etc.
Each method can be called with or without an Appearance. If called without an Appearance, the shape will be colored black. 
In order to make creating different Appearances easy, we've added a class called YoAppearance which contains static methods for creating common colors and appearances.

Adds a shape of the given Geometry and Appearance. Geometry and Appearance are classes in the Java3D API. Refer to a Java3D reference manual for more information.
```java
void addShape(Geometry, Appearance)
```

Adds a cube with lengths lx, ly, lz. Origin is at the center of the base of the cube (not at the center of mass of the cube)
```java
void addCube(double lx, double ly, double lz) 
void addCube(double lx, double ly, double lz, Appearance)
```

Adds a wedge (a cube cut diagonally in half) with lengths lx, ly, lz. Origin is at the center of the base of the wedge.
```java
void addWedge(double lx, double ly, double lz) 
void addWedge(double lx, double ly, double lz, Appearance)
```

Adds a sphere of the given radius. Origin is at the center of the sphere.
```java
void addSphere(double radius)
void addSphere(double radius, Appearance)
```

Adds an ellipsoid (sphere with 3 different radii). Origin is at the center of the ellipsoid.
```java
public void addEllipsoid(double xRad, double yRad, double zRad) 
public void addEllipsoid(double xRad, double yRad, double zRad, Appearance)
```

Adds a hemi-ellipsoid (top half of an ellipsoid). The radii of the base is xRad and yRad. The height is zRad. Origin is at the center of the base of the hemi-ellipsoid.
```java
public void addHemiEllipsoid(double xRad, 
double yRad, double zRad) 
public void addHemiEllipsoid(double xRad, double yRad, double zRad, Appearance)
```

Adds a cylinder with the given height and radius. Origin is at the center of the base of the cylinder (not at the center of mass of the cylinder)
```java
void addCylinder(double height, double radius) 
void addCylinder(double height, double radius, Appearance)
```

Adds a cone of given height and radius. Origin is at the center of the base of the cone.
```java
void addCone(double height, double radius) 
void addCone(double height, double radius, Appearance)
```

Adds a general truncated cone with the given height. The base is an ellipse with radii of bx and by. The top is an ellipse with radii of tx and ty. Origin is at the center of the base of the cone.
```java
void addGenTruncatedCone(double height, double bx, double by, double tx, double ty) 
void addGenTruncatedCone(double height, double bx, double by, double tx, double ty, Appearance)
```

Adds a section of a torus, from startAngle to endAngle. The majorRadius is the distance from the center to the center of the torus. The minor radius is the radius of the torus itself. Origin is at the center of the torus.
```java
void addArcTorus(double startAngle, double endAngle, double majorRadius, double minorRadius) 
void addArcTorus(double startAngle, double endAngle, double majorRadius, double minorRadius, Appearance)
```

Adds a 12 sided cube with pyramids on both ends. The cube has sides of length lx,ly, and lz. The pyramids are each of height lh. Origin is at the center of mass.
```java
void addPyramidCube(double lx, double ly, double lz, double lh) 
void addPyramidCube(double lx, double ly, double lz, double lh, Appearance)
```

Adds the geometry specified in the 3D Studio Max (3ds) file. Includes the texture mapping, but requires the texture files to be located with the 3ds file.
```java
void add3DSFile(String fileName) 
void add3DSFile(String fileName, Appearance app) 
add3DSFile(URL fileURL, Appearance app)
```

Adds the geometry specified in the VRML file. Sets the Appearance to that specified. If no Appearance is specified, then uses the Appearance in the VRML file. Note: Only works for VRML Version 2.0 and not Version 1.0
```java
void addVRMLFile(URL fileURL) 
void addVRMLFile(URL fileURL, Appearance)
```

Adds a coordinate system with rods of the given length. The x axis is red, the y axis is white, and the z axis is blue.
```java
void addCoordinateSystem (double length)
```

### Related Example:

[Example of Adding Shapes](https://ihmcrobotics.github.io/simulation-construction-set/docs/04-yo-appearance-api.html)