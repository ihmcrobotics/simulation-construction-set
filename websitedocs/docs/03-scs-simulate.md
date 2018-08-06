---
title: Simulate methods
---

```java
public void simulate()
```
Simulate continuously.

```java
public void simulate(double simulationTime)
```
Simulate for simulationTime time.

```java
public void simulate(int numTicks)
```
Simulate for numTicks ticks.

```java
public void addSimulateDoneListener(SimulateDoneListener listener)
```
Adds a SimulateDoneListener that is called back when simulate() is completed. SimulateDoneListener is an interface whose sole method is public void simulateDone()

```java
public void stop()
```
Stop simulating.