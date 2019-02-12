package us.ihmc.scs

import org.gradle.api.Plugin
import org.gradle.api.Project
import org.gradle.api.tasks.JavaExec
import org.gradle.api.tasks.application.CreateStartScripts
import org.gradle.api.tasks.testing.Test

class SCSPlugin : Plugin<Project>
{
   var scsShowGui = false
   var scsVideo = false
   var runningOnCIServer = false
   val javaProperties = hashMapOf<String, String>()
   lateinit var project: Project

   open class SCSExtension(val javaProperties: HashMap<String, String>,
                           val scsPlugin: SCSPlugin)  // in the future, add real fields?
   {
      fun showGui()  // these methods provide the ability to configure dynamically in build.gradle
      {
         if (!scsPlugin.scsShowGui)
         {
            javaProperties["show.scs.windows"] = "true"
            javaProperties["create.scs.gui"] = "true"
            javaProperties["java.awt.headless"] = "false"
            javaProperties["show.scs.yographics"] = "true"
            javaProperties["show.splash.screen"] = "true"

            scsPlugin.putProperties()
         }
      }

      fun captureVideo()
      {
         if (!scsPlugin.scsVideo)
         {
            javaProperties["show.scs.windows"] = "true"
            javaProperties["create.scs.gui"] = "true"
            javaProperties["java.awt.headless"] = "false"
            javaProperties["show.scs.yographics"] = "true"
            javaProperties["show.splash.screen"] = "true"
            javaProperties["show.scs.yographics"] = "false"
            javaProperties["create.scs.videos"] = "true"
            if (scsPlugin.runningOnCIServer)
               javaProperties["create.videos.dir"] = "/opt/ihmc/bamboo-videos"
            else
               javaProperties["create.videos.dir"] = System.getProperty("user.home") + "/bamboo-videos"

            scsPlugin.putProperties()
         }
      }
   }

   override fun apply(project: Project)
   {
      this.project = project

      project.properties["scsVideo"].run { if (this != null) scsVideo = (this as String).toBoolean() }
      project.properties["scsShowGui"].run { if (this != null) scsShowGui = (this as String).toBoolean() }
      System.getenv("RUNNING_ON_CONTINUOUS_INTEGRATION_SERVER").run { if (this != null) runningOnCIServer = (this as String).toBoolean() }

      // defaults
      javaProperties["keep.scs.up"] = "false"
      javaProperties["run.multi.threaded"] = "true"
      javaProperties["use.perfect.sensors"] = "false"
      javaProperties["scs.dataBuffer.size"] = "8142"
      javaProperties["openh264.license"] = "accept"
      javaProperties["disable.joint.subsystem.publisher"] = "true"
      javaProperties["check.nothing.changed.in.simulation"] = "false"
      javaProperties["show.scs.windows"] = "false"
      javaProperties["create.scs.gui"] = "false"
      javaProperties["show.scs.yographics"] = "false"
      javaProperties["java.awt.headless"] = "true"
      javaProperties["create.videos"] = "false"
      javaProperties["show.splash.screen"] = "false"
      if (scsShowGui || scsVideo)
      {
         javaProperties["show.scs.windows"] = "true"
         javaProperties["create.scs.gui"] = "true"
         javaProperties["java.awt.headless"] = "false"
         javaProperties["show.scs.yographics"] = "true"
         javaProperties["show.splash.screen"] = "true"
      }
      if (scsVideo)
      {
         javaProperties["show.scs.yographics"] = "false"
         javaProperties["create.scs.videos"] = "true"
         if (runningOnCIServer)
            javaProperties["create.videos.dir"] = "/opt/ihmc/bamboo-videos"
         else
            javaProperties["create.videos.dir"] = System.getProperty("user.home") + "/bamboo-videos"
      }

      project.extensions.create("scs", SCSExtension::class.java, javaProperties, this)

      putProperties()
   }

   private fun putProperties()
   {
      for (allproject in project.allprojects)
      {
         allproject.tasks.withType(JavaExec::class.java) { javaExec ->
            // setup properties for all JavaExec tasks
            javaExec.systemProperties.putAll(javaProperties)
            allproject.logger.info("[scs] Passing JVM args ${javaExec.systemProperties} to $javaExec")
         }
         allproject.tasks.withType(Test::class.java) { test ->
            // setup properties for forked test jvms
            test.systemProperties.putAll(javaProperties)
            allproject.logger.info("[scs] Passing JVM args ${test.systemProperties} to $test")
         }
         allproject.tasks.withType(CreateStartScripts::class.java) { startScripts ->
            // setup properties for all start scripts (includes application plugin)
            val list = arrayListOf<String>()
            javaProperties.forEach {
               list.add("-D${it.key}=${it.value}")
            }
            startScripts.defaultJvmOpts = list
            allproject.logger.info("[scs] Passing JVM args $list to $startScripts")
         }
      }
   }
}