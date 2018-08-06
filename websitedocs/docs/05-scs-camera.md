---
title: Camera Methods
---
<a name="scsCameraAPI">

```java
void setCameraFix(double fixX, double fixY, double fixZ)
```
Sets the coordinates of the position that the camera is fixed to (looking at).

```java
void setCameraTracking(boolean track, boolean trackX, boolean trackY, boolean trackZ)
```
Sets whether the camera is tracking an object or not. Tracking along individual axes (x,y,z) can be set.

```java
void setCameraTrackingOffsets(double dx, double dy, double dz)
```
Sets the offset vector of the fix point of the camera from the object it is tracking.

```java
void setCameraTrackingVars(String xName, String yName, String zName)
```
Sets the variable names of the object the camera is tracking. Default is q_x, q_y, q_z.

```java
void setCameraPosition(double posX, double posY, double posZ)
```
Sets the coordinates of the position of the camera.

```java
void setCameraDolly(boolean dolly, boolean dollyX, boolean dollyY, boolean dollyZ)
```
Sets whether the camera is dollying relative to an object or not. Dolly along individual axes (x,y,z) can be set.

```java
void setCameraDollyOffsets(double dx, double dy, double dz)
```
Sets the camera offset vector from the object it is dollying relative to.

```java
void setCameraDollyVars(String xName, String yName, String zName)
```
Sets the variable names of the object the camera is dollying relative to. Default is q_x, q_y, q_z.

### Related Example:

[Example of Camera Methods](https://ihmcrobotics.github.io/simulation-construction-set/docs/01-creating-a-new-project.html)