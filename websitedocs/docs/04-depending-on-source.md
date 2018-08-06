---
title: Depending Directly on the Source
---

For the IHMCOpenRoboticsSoftware and [ihmc-build](https://github.com/ihmcrobotics/ihmc-build) to work correctly when depending directly on the source, the Gradle project hierarchy needs to take a particular form. Let's assume you have a directory structure such as:

    .<your Gradle root project>
    ├── build.gradle
    ├── settings.gradle
    ├── ProjectA
    ├── ProjectB
    ├── IHMCOpenRoboticsSoftware
    │   └── Acsell
    │   └── Atlas
    │   └── CommonWalkingControlModules
    ├── ProjectC
    └── ...

You will need to modify the **root project** `settings.gradle` to set up your project hierarchy correctly. The important thing is that you will need both `:IHMCOpenRoboticsSoftware` as well as the various `:IHMCOpenRoboticsSoftware:<Sub Project>` items in the Gradle hierarchy, even though IHMCOpenRoboticsSoftware doesn't contain any of its own Java source. This can be tedious to set up by hand, so we usually write our settings.gradle to do some naive dynamic generation of dependencies.

An example `settings.gradle` similar to what our developers use could look like the following:

```groovy
rootProject.name = '_Foo' // Your root project name goes here

def codeDir = rootProject.projectDir

def isGradleProjectFilter = new FilenameFilter() {
    @Override
    boolean accept(File dir, String name) {
        File f = new File(dir, name).getCanonicalFile();
        return f.isDirectory() && new File(f, "build.gradle").exists();
    }
}

codeDir.list(isGradleProjectFilter).each { mainProject ->
    include "${mainProject}"

    project(":${mainProject}").projectDir.list(isGradleProjectFilter).each { subProject ->
        include "${mainProject}:${subProject}"
    }
}
```

If this is set up correctly, you can either apply the `ihmc-build` plugin from the [Plugin portal](https://plugins.gradle.org/plugin/us.ihmc.gradle.ihmc-build) and use the dependency resolver methods exposed by the build extensions, or you can manually identify dependencies on projects using the normal Gradle syntax for project dependencies. A sample build.gradle dependency block:

```groovy
dependencies {
  compile project(':IHMCOpenRoboticsSoftware:IHMCJavaToolkit') // normal Gradle way of doing things
}

/* OR */

dependencies {
  compile ihmc.getProjectDependency(':IHMCJavaToolkit') // ihmc-build way of doing things
}
```
