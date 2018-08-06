---
title: Create the World
---

```java
void addStaticBranchGroup(BranchGroup staticBranchGroup)
```
Adds a static branch group with no physics. For creating the surrounding static world.

```java
void setupVarGroup(String name, String[] varNames)
void setupVarGroup(String name, String[] varNames, String[] regularExpressions)
```
Defines a group of variables that can be selected via the GUI. The first String array contains exact names. The second optional String array is regular expressions that will be matched to YoVariable names.

```java
void setupGraphGroup(String name, String[][] varNames)
```
Defines a graph group that can be selected via the GUI.

```java
void setupEntryBoxGroup(String name, String[] varNames)
```
Defines a group of entry boxes that can be selected via the GUI.

```java
void setupConfiguration(String name, String varGroup, String graphGroup, String entryBoxGroup)
```
Defines a configuration consisting of a variable group, a graph group, and an entry box group. Use "all" for the varGroup to display all the variables.

```java
void selectConfiguration(String name)
```
Selects the default configuration for the GUI.

```java
public void setGroundVisible(boolean isVisible)
```
Makes the ground invisible if isVisible is false, otherwise makes the ground visible.

```java
void setGroundAppearance(Appearance app)
```
Sets the Appearance of the ground to app.