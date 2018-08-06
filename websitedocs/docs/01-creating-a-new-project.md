---
title: Create a New Project
---

First you need to create the `SimulationConstructionSetTutorial` *Gradle project* which references some IHMC Maven artifacts.  This project will also be used by all the other SCS tutorials. If you need a reminder of how to create the project, checkout the [Quick Start](https://ihmcrobotics.github.io/ihmc-open-robotics-software/docs/quickstarthome.html).

### 1. Create the Project Directory Structure and the build.gradle File

Create a directory called `SimulationConstructionSetTutorial` and create the Gradle folder structure as follows:

    SimulationConstructionSetTutorial
    └── src
        └── main
            └── java

### 2. Create the build.gradle File

In your `SimulationConstructionSetTutorial` folder create a file named `build.gradle` with the following contents:

```groovy

apply plugin: 'java'  

sourceCompatibility = 1.5  

repositories {
    maven {
        url  "http://dl.bintray.com/ihmcrobotics/maven-release" // IHMC Code releases
    }

    maven {
        url  "http://dl.bintray.com/ihmcrobotics/maven-vendor" // Third-party libraries that we have vendored for various reasons
    }

    /*
     *  Maven repos hosted at IHMC for some legacy vendored
     *  dependencies we have not been able to vendor on Bintray yet.
     *  This will be going away eventually.
     */
    maven {
        url "https://bengal.ihmc.us/nexus/content/repositories/thirdparty/"
    }

    jcenter() // One of the central Maven repos. You can also use mavenCentral() instead or in addition to.
}

dependencies {
    compile 'us.ihmc:SimulationConstructionSet:{{OpenSourceVersion}}' 
}

```
<br>
Now import this Gradle project into your IDE as you did in the Quick Start.


