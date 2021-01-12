import com.gradle.publish.MavenCoordinates

plugins {
   `java-gradle-plugin`
   kotlin("jvm") version "1.4.10"
   id("com.gradle.plugin-publish") version "0.12.0"
}

group = "us.ihmc"
version = "0.5"

repositories {
   mavenCentral()
   jcenter()
}

dependencies {
   api(gradleApi())
   api(kotlin("stdlib"))
}

val pluginDisplayName = "SCS Gradle Plugin"
val pluginDescription = "Runtime configuration for IHMC's Simulation Construction Set."
val pluginVcsUrl = "https://github.com/ihmcrobotics/simulation-construction-set"
val pluginTags = listOf("scs", "property", "initialization", "ihmc", "robotics", "simulation")

gradlePlugin {
   plugins.register(project.name) {
      id = project.group as String + "." + project.name
      implementationClass = "us.ihmc.scs.SCSPlugin"
      displayName = pluginDisplayName
      description = pluginDescription
   }
}

pluginBundle {
   website = pluginVcsUrl
   vcsUrl = pluginVcsUrl
   description = pluginDescription
   tags = pluginTags

   plugins.getByName(project.name) {
      id = project.group as String + "." + project.name
      version = project.version as String
      displayName = pluginDisplayName
      description = pluginDescription
      tags = pluginTags
   }

   mavenCoordinates(closureOf<MavenCoordinates> {
      groupId = project.group as String
      artifactId = project.name
      version = project.version as String
   })
}