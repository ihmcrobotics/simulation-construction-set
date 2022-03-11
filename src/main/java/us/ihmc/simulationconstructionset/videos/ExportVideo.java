package us.ihmc.simulationconstructionset.videos;

import java.awt.Dimension;
import java.awt.image.BufferedImage;
import java.io.File;
import java.io.IOException;
import java.util.Vector;

import us.ihmc.codecs.builder.H264Settings;
import us.ihmc.codecs.builder.MP4H264MovieBuilder;
import us.ihmc.codecs.generated.EProfileIdc;
import us.ihmc.codecs.generated.EUsageType;
import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.jMonkeyEngineToolkit.Graphics3DAdapter;
import us.ihmc.jMonkeyEngineToolkit.camera.CameraController;
import us.ihmc.jMonkeyEngineToolkit.camera.CaptureDevice;
import us.ihmc.jMonkeyEngineToolkit.camera.ViewportAdapter;
import us.ihmc.log.LogTools;
import us.ihmc.simulationconstructionset.GotoInPointCommandExecutor;
import us.ihmc.simulationconstructionset.GotoOutPointCommandExecutor;
import us.ihmc.simulationconstructionset.TimeHolder;
import us.ihmc.simulationconstructionset.commands.ExportVideoCommandExecutor;
import us.ihmc.simulationconstructionset.commands.RunCommandsExecutor;
import us.ihmc.simulationconstructionset.gui.ActiveCanvas3DHolder;
import us.ihmc.simulationconstructionset.gui.StandardSimulationGUI;
import us.ihmc.simulationconstructionset.gui.dialogConstructors.GUIEnablerAndDisabler;
import us.ihmc.simulationconstructionset.synchronization.SimulationSynchronizer;
import us.ihmc.simulationconstructionset.util.XMLReaderUtility;
import us.ihmc.yoVariables.buffer.interfaces.YoBufferReader;

public class ExportVideo implements ExportVideoCommandExecutor
{
   private final static boolean DEBUG = false;

   private final TimeHolder timeHolder;
   private final StandardSimulationGUI standardSimulationGUI;

   private final SimulationSynchronizer simulationSynchronizer;

   private final YoBufferReader dataBufferCommandsExecutor;
   private final GUIEnablerAndDisabler guiEnablerAndDisabler;
   private final RunCommandsExecutor runCommandsExecutor;

   private final GotoInPointCommandExecutor gotoInPointCommandExecutor;
   private final GotoOutPointCommandExecutor gotoOutPointCommandExecutor;

   public ExportVideo(TimeHolder timeHolder,
                      StandardSimulationGUI standardSimulationGUI,
                      YoBufferReader dataBufferCommandsExecutor,
                      GotoInPointCommandExecutor gotoInPointCommandExecutor,
                      GotoOutPointCommandExecutor gotoOutPointCommandExecutor,
                      RunCommandsExecutor runCommandsExecutor,
                      GUIEnablerAndDisabler guiEnablerAndDisabler,
                      ActiveCanvas3DHolder activeCanvas3DHolder,
                      SimulationSynchronizer simulationSynchronizer)
   {
      this.timeHolder = timeHolder;
      this.gotoInPointCommandExecutor = gotoInPointCommandExecutor;
      this.gotoOutPointCommandExecutor = gotoOutPointCommandExecutor;
      this.simulationSynchronizer = simulationSynchronizer;
      this.standardSimulationGUI = standardSimulationGUI;

      this.dataBufferCommandsExecutor = dataBufferCommandsExecutor;
      this.runCommandsExecutor = runCommandsExecutor;
      this.guiEnablerAndDisabler = guiEnablerAndDisabler;
   }

   @Override
   public void createVideo(File selectedFile)
   {
      Dimension dimension = new Dimension(1280, 720); // Default to 720p
      //      Dimension dimension = new Dimension(1920, 1080); // Default to 1080p

      Boolean isSequanceSelected = false;
      double playBackRate = 1.0;
      double frameRate = 30.0;

      CameraController cameraController = standardSimulationGUI.getActiveView().getCameraController();

      this.createVideo(cameraController, selectedFile, dimension, isSequanceSelected, playBackRate, frameRate);
   }

   @Override
   public void createVideo(CameraController cameraController,
                           File selectedFile,
                           Dimension dimension,
                           Boolean isSequanceSelected,
                           double playBackRate,
                           double frameRate)
   {
      Graphics3DAdapter graphics3dAdapter = standardSimulationGUI.getGraphics3dAdapter();

      ViewportAdapter adapter = graphics3dAdapter.createNewViewport(null, false, true);

      adapter.setupOffscreenView((int) dimension.getWidth(), (int) dimension.getHeight());

      adapter.setCameraController(cameraController);

      CaptureDevice captureDevice = adapter.getCaptureDevice();
      this.createVideo(captureDevice, selectedFile, false, playBackRate, frameRate);

      graphics3dAdapter.closeViewport(adapter);
   }

   @Override
   public void createVideo(CaptureDevice captureDevice, File selected, Boolean isSequenceSelected, double playBackRate, double frameRate)
   {
      printIfDebug("Creating Video. File = " + selected);

      int currentTick = 1;
      File selectedFile = selected;

      double realTimePlaybackRate = runCommandsExecutor.getPlaybackRealTimeRate();

      guiEnablerAndDisabler.disableGUIComponents();
      runCommandsExecutor.setPlaybackRealTimeRate(1.0);
      String fileName = selectedFile.getName();

      if (!fileName.contains("."))
      {
         fileName = fileName.concat(".mov");

         if (!selectedFile.getName().equals(fileName))
         {
            File newChosenFile = new File(selectedFile.getParent(), fileName);

            selectedFile = newChosenFile;
         }
      }

      // stop the simulation
      runCommandsExecutor.stop();
      ThreadTools.sleep(200);
      gotoOutPointCommandExecutor.gotoOutPoint();
      ThreadTools.sleep(200);

      // go to the start
      gotoInPointCommandExecutor.gotoInPoint();

      // sleep for a little
      ThreadTools.sleep(700);

      // record the start tick
      currentTick = dataBufferCommandsExecutor.getInPoint();

      // TICKS_PER_PLAY_CYCLE = sim.getTicksPerPlayCycle();

      String fileNameNoExtension = fileName.substring(0, XMLReaderUtility.getEndIndexOfSubString(0, fileName, ".") - 1);
      dataBufferCommandsExecutor.setCurrentIndex(currentTick);

      try
      {
         Thread.sleep(200);
      }
      catch (InterruptedException e)
      {
      }

      Graphics3DAdapter graphics3dAdapter = standardSimulationGUI.getGraphics3dAdapter();
      graphics3dAdapter.play();

      if (isSequenceSelected)
      {
         saveSimulationAsSequenceOfImages(selectedFile.getParent(), fileNameNoExtension, captureDevice);
         guiEnablerAndDisabler.enableGUIComponents();
         System.out.println("Finished making saving sequence of Images.");

         return;
      }
      else
      {
         videoPlaybackAsBufferedImage(selectedFile.getAbsolutePath(), captureDevice, playBackRate, frameRate);
      }

      graphics3dAdapter.pause();

      runCommandsExecutor.setPlaybackRealTimeRate(realTimePlaybackRate);
      guiEnablerAndDisabler.enableGUIComponents();

   }

   private void printIfDebug(String message)
   {
      if (DEBUG)
         System.out.println(message);
   }

   public void videoPlaybackAsBufferedImage(String file, CaptureDevice captureDevice, double playBackRate, double frameRate)
   {

      // *****************************
      // long nextWakeMillis;
      // long currentTime;

      standardSimulationGUI.updateGraphs();
      ThreadTools.sleep(100);

      gotoInPointCommandExecutor.gotoInPoint();
      ThreadTools.sleep(100);
      
      double startTime = timeHolder.getTime();
      dataBufferCommandsExecutor.tickAndReadFromBuffer(1);
      ThreadTools.sleep(100);

      // Compute the DT at which the data is recorded in the buffer. That'll allow us to navigate the buffer more quickly.
      double recordDT = timeHolder.getTime() - startTime;
      
      gotoOutPointCommandExecutor.gotoOutPoint();
      ThreadTools.sleep(100);
      
      double endTime = timeHolder.getTime();
      
      gotoInPointCommandExecutor.gotoInPoint();
      ThreadTools.sleep(100);

      double lastFrameTime = timeHolder.getTime(); // That's the time of the first frame that'll be exported. 
      if (DEBUG)
         System.out.println("Start time: " + lastFrameTime);

      double videoDT = playBackRate / frameRate;
      BufferedImage bufferedImage = captureDevice.exportSnapshotAsBufferedImage();

      MP4H264MovieBuilder movieBuilder = null;
      try
      {
         H264Settings settings = new H264Settings();
         settings.setBitrate(bufferedImage.getWidth() * bufferedImage.getHeight() / 100);
         settings.setUsageType(EUsageType.CAMERA_VIDEO_REAL_TIME);
         settings.setProfileIdc(EProfileIdc.PRO_HIGH);

         movieBuilder = new MP4H264MovieBuilder(new File(file), bufferedImage.getWidth(), bufferedImage.getHeight(), (int) frameRate, settings);

         boolean reachedEndPoint = false; // This keeps track of what the previous index was to stop the playback when it starts to loop back.

         
         int frameIndex = 0;
         while (!reachedEndPoint)
         {
            printIfDebug("ExportVideo: Capturing Frame");

            movieBuilder.encodeFrame(captureDevice.exportSnapshotAsBufferedImage());
            frameIndex++;

            printIfDebug("Waiting For simulationSynchronizer 1");
            synchronized (simulationSynchronizer) // Synched so we don't update during a graphics redraw...
            {
               printIfDebug("Done Waiting For simulationSynchronizer 1");

               double nextFrameTime = lastFrameTime + videoDT;
               int stepSize = (int) Math.round((nextFrameTime - timeHolder.getTime()) / recordDT);
               reachedEndPoint = dataBufferCommandsExecutor.tickAndReadFromBuffer(stepSize);

               if (reachedEndPoint)
               {
                  if(frameIndex < 100)
                  {
                     System.out.println("reachedEndPoint: t=" + timeHolder.getTime() + ", dt=" + (timeHolder.getTime() - lastFrameTime) + " stepSize: " + stepSize);
                     LogTools.warn("Something is probably wrong with the exported video!");
                  }
                  break;
               }

               standardSimulationGUI.updateRobots();
               standardSimulationGUI.updateGraphs();
               standardSimulationGUI.allowTickUpdatesNow();
               if (DEBUG)
                  System.out.println("ExportVideo: Capturing Frame at t=" + timeHolder.getTime() + ", dt=" + (timeHolder.getTime() - lastFrameTime));
               lastFrameTime = nextFrameTime;
            }

            //         if (sim.isGraphsUpdatedDuringPlayback())
            standardSimulationGUI.updateGraphs();
         }
      }
      catch (IOException e)
      {
         LogTools.error("Could not crate movie.  " + e.getMessage());
      }
      finally
      {
         if (movieBuilder != null)
         {
            try
            {
               movieBuilder.close();
            }
            catch (IOException e)
            {
            }
         }
      }

      if (DEBUG)
         LogTools.info("Done with video");
   }

   public Vector<File> saveSimulationAsSequenceOfImages(String path, String NameNoExtension, CaptureDevice captureDevice)
   {
      standardSimulationGUI.updateGraphs();

      try
      {
         Thread.sleep(125);
      }
      catch (InterruptedException e)
      {
      }

      Vector<File> output = new Vector<>();

      int last = 0; // This keeps track of what the previous index was to stop the playback when it starts to loop back.
      if (standardSimulationGUI == null)
         return null; // Only works with a GUI

      gotoInPointCommandExecutor.gotoInPoint();

      while (last < dataBufferCommandsExecutor.getOutPoint())
      {
         last = dataBufferCommandsExecutor.getCurrentIndex();

         File file = new File(path, NameNoExtension + "_" + last + ".jpeg");

         captureDevice.exportSnapshot(file);
         output.add(file);

         printIfDebug("Waiting For simulationSynchronizer 2");

         synchronized (simulationSynchronizer) // Synched so we don't update during a graphics redraw...
         {
            printIfDebug("Done Waiting For simulationSynchronizer 2");

            dataBufferCommandsExecutor.tickAndReadFromBuffer(1);
            standardSimulationGUI.updateRobots();
            standardSimulationGUI.allowTickUpdatesNow();
         }

         //         if (updateGraphs)
         standardSimulationGUI.updateGraphs();
      }

      return output;
   }

   // /**
   // * This is almost exactly like playCycle() but it also saves the screenShots of each frame as it plays the simulation.
   // */
   // public Vector<BufferedImage> moviePlaybackAsBufferedImage(YoCanvas3D canvas3D)
   // {
   // myGUI.updateGraphs();
   //
   // try
   // {
   // Thread.sleep(125);
   // }
   // catch (InterruptedException e)
   // {
   // }
   //
   // nextWakeMillis = System.currentTimeMillis();
   // Vector<BufferedImage> output = new Vector<BufferedImage>();
   // last = 0;    // This keeps track of what the previous index was to stop the playback when it starts to loop back.
   // if (myGUI == null)
   // return null;    // Only works with a GUI
   // myDataBuffer.goToInPoint();
   // while (last <= myDataBuffer.getOutPoint())
   // {
   // System.out.println("Here C");
   // last = myDataBuffer.ordinal();
   //
   // //        while ((currentTime = System.currentTimeMillis()) < nextWakeMillis)
   // //        {
   // //           try
   // //           {
   // //              Thread.sleep(5);
   // //           }
   // //           catch (InterruptedException e)
   // //           {
   // //           }
   // //
   // //           // Thread.yield();
   // //        }
   //
   //
   // //      exportSnapshot(file);
   // //      exportSn
   // output.add(exportSnapshotAsBufferedImage(canvas3D));
   //
   // //
   // //        long numTicks = (currentTime - nextWakeMillis) / ((PLAY_CYCLE_TIME_MS)) + 1;
   // //
   // //        nextWakeMillis = nextWakeMillis + ((PLAY_CYCLE_TIME_MS)) * numTicks;
   //
   // // myDataBuffer.tick(Math.max((int) (TICKS_PER_PLAY_CYCLE * numTicks), 1));
   //
   // // myGUI.allowTickUpdatesNow();
   //
   // synchronized (robots)    // Synched so we don't update during a graphics redraw...
   // {
   // myDataBuffer.tick(1);
   // myGUI.updateRobots();
   // myGUI.allowTickUpdatesNow();
   // }
   //
   // if (updateGraphs)
   // myGUI.updateGraphs();
   // }
   //
   // return output;
   // }

}
