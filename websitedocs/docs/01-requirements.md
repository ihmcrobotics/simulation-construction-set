---
title: Requirements
---

*IHMCOpenRoboticsSoftware* uses the [Gradle build system](https://ihmcrobotics.github.io/docs/installgradle.html), and requires [JDK 8](https://ihmcrobotics.github.io/docs/installjava.html).

Gradle is a build system for Java. It has the benefit of being able to integrate with other Java technology that is used for dependency resolution like Ivy and Maven; if you're coming from a C/C++ world, you can think of it as CMake with a built-in package manager. Currently, we require **Gradle 2.10+**. We provide a versioned [Gradle wrapper](https://docs.gradle.org/current/userguide/gradle_wrapper.html) for getting started quickly. The Gradle wrapper will always reflect the minimum version of Gradle required to build the software; if we adopt features only present in newer versions of Gradle as they are release we will update the wrapper.

We also strongly suggest an IDE, either [Eclipse Mars.1+](http://www.eclipse.org/downloads/packages/eclipse-ide-java-developers/mars2) or [IntelliJ IDEA 14+](https://www.jetbrains.com/idea/download/) (Ultimate or Community is fine).

Required:

* [Java 8u111 or higher](https://ihmcrobotics.github.io/docs/installjava.html)
* [Gradle 2.10+](https://ihmcrobotics.github.io/docs/installgradle.html)