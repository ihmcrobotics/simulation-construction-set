---
title: Using IHMC Open Robotics Software .jar releases with Maven/Gradle
---

*IHMCOpenRoboticsSoftware* binary releases are provided as Maven artifacts. Gradle can interact with *Maven repositories*, so we give the gradle script the URLs for a few different repositories, and then we can specify dependencies using the Maven "Group Artifact Version (GAV)" nomenclature. Packages are identified by their "group", a namespace that may contain several packages, followed by the "artifact", or the name of the package we want to pull in, and then the "version" of the package that we want.

You can browse our Maven repository and download .jar files directly at <https://bintray.com/ihmcrobotics/maven-release>

As a nice bonus, all Maven artifacts maintain a descriptor (called a POM) that stores information about that artifact's dependencies. So unless you need to have control over which versions of a library you're using, when you specify a Maven artifact that you want to depend on you don't need to specify all of its dependencies by hand; they'll get resolved automatically by the build system as it inspects each artifact's POM!

At a minimum, you will need to have the following repositories declared in your `gradle.build` script to use the *IHMCOpenRoboticsSoftware* .jars:

```groovy

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

   /* You will also need to add either jcenter() or mavenCentral() or both, depending on your preference */
}

```
