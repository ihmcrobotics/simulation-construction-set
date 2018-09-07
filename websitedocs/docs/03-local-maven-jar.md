---
title: Building .jars
---

IHMCOpenRoboticsSoftware is pre-configured for generating Maven publications. You can publish directly from the source code right in to your local Maven repository, e.g. the `$HOME/.m2` directory. These builds will be tagged with a build "version" of `"LOCAL"` instead of an incrementing version number.

An example workflow for developing against a local clone of the software:

1. Clone IHMCOpenRoboticsSoftware
2. Make modifications
3. Publish to your local `$HOME/.m2` repository

**To publish jars to your local Maven repository:**  
```groovy
$ cd <IHMCOpenRoboticsSoftware directory>
$ ./gradlew publishAllToMavenLocal
```

**To depend on the jars in your local Maven repository:**

In this example we'll have a compile-time dependency of the locally built SimulationConstructionSet project. In the `build.gradle` of the project you wish to have link against SimulationConstructionSet:

```bash
repositories {
  mavenLocal()
  <your other repositories>
}

dependencies {
  compile group: 'us.ihmc', name: 'SimulationConstructionSet', version: 'LOCAL', changing: true
}
```