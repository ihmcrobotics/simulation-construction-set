---
title: Simulation Parameters
---

```java
void setDT(double simulateDT, int recordFrequency) 
double getDT()
```
Sets the integration time step and variable record frequency (steps per record). 
Defaults are 0.0004 seconds and 50 steps per record (0.02 seconds per record).

```java
void setPlaybackRealTimeRate(double rate)
```
Sets the playback slow-motion/fast-motion rate. The default value of 1.0 means real time playback.

```java
void setPlaybackDesiredFrameRate(double rate)
```
Sets the desired graphics update rate during playback. Default value is 0.04, or 25 fps.

```java
YoVariable getVar(String varname)
```
Finds and returns a YoVariable of the given name.

```java
void addStaticLink(Link staticLink)
```
Adds a static link with no physics. For creating the surrounding static world.

```java
public void setFastSimulate(boolean fastSimulate)
```
If fastSimulate is true, makes the GUI less responsive during simulation in order to increase simulation speed.

### Related Example:

[Example of Simulation Parameters](https://ihmcrobotics.github.io/simulation-construction-set/docs/000-summary.html)