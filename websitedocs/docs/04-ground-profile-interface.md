---
title: GroundProfile Interface
---

GroundProfile is an interface. By setting a GroundProfile to your GroundContactModel, and by setting your GroundContactModel to your robot, the ground when the simulation is run will be drawn as the ground profile.
The user can generate his/her own GroundProfile, or can use one of the GroundProfiles provided in the com.yobotics.simulationconstructionset.utils package. 
If no GroundProfile is specified, then a flat ground will be simulated.

Returns the height of this ground profile, given the x, y, and z values.
```java
public abstract double heightAt(double x, double y, double z);
```

Returns true if the ground contact point has penetrated this ground profile. Otherwise returns either true or false. Used only for efficiency, so it is always safe to return true.
```java
public abstract boolean isClose(double x, double y, double z);
```

Sets the point intersection to be the closest intersection between the ground profile and the point at (x,y,z). If computation of the intersection is too difficult, return (x, y, heightAt(x,y,z)).
```java
public abstract void closestIntersectionTo (double x, double y, double z, Point3d intersection);
```

Sets the vector normal to be the surface normal of the ground at point (x,y,z). Straight up is the vector (0,0,1)
```java
public abstract void surfaceNormalAt(double x, double y, double z, Vector3d normal);
```

Sets both the point intersection and surface normal. This method is redundant, but exists due to potential efficiency improvements by computing the intersection and surface normal at the same time. It is safe to implement this method by calling both closestIntersectionTo() and surfaceNormalAt()
```java
public abstract void closestIntersectionAndNormalAt(double x, double y, double z, Point3d intersection, Vector3d normal);
```

Returns the boundaries of the ground profile. Everything outside this bound will be drawn at a height of zero. Inside this bound, the ground will be drawn as per the profile. The smaller the bound, the more detailed the graphics. This bound does not effect the resolution of the ground physics, however, which is only bounded by the precision of the numbers.
```java
public abstract double getXMin(); 
public abstract double getXMax(); 
public abstract double getYMin(); 
public abstract double getYMax();
```

Returns the number of tilings to perform in both the x and y direction if the ground is texture mapped.
```java
public abstract double getXTiles(); 
public abstract double getYTiles();
```

### Related Example: 

[Example of GroundProfile in Use](https://ihmcrobotics.github.io/simulation-construction-set/docs/04-wavy-ground-profile.html)