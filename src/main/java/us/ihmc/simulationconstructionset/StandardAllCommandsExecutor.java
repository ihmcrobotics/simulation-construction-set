package us.ihmc.simulationconstructionset;

import java.awt.Dimension;
import java.awt.Point;
import java.io.File;
import java.util.ArrayList;
import java.util.List;

import org.apache.commons.lang3.StringUtils;

import us.ihmc.simulationconstructionset.commands.AllCommandsExecutor;
import us.ihmc.simulationconstructionset.commands.ViewportSelectorCommandListener;
import us.ihmc.simulationconstructionset.gui.EventDispatchThreadHelper;
import us.ihmc.simulationconstructionset.gui.GUIConfigurationSaveAndLoad;
import us.ihmc.simulationconstructionset.gui.GraphArrayWindow;
import us.ihmc.simulationconstructionset.gui.StandardSimulationGUI;
import us.ihmc.simulationconstructionset.gui.ViewportWindow;
import us.ihmc.simulationconstructionset.gui.YoVariableSliderWindow;
import us.ihmc.yoVariables.buffer.YoBuffer;
import us.ihmc.yoVariables.buffer.interfaces.KeyPointsChangedListener;

public class StandardAllCommandsExecutor implements AllCommandsExecutor
{
   private StandardSimulationGUI standardSimulationGUI;
   private SimulationConstructionSet simulationConstructionSet;
   private YoBuffer dataBuffer;

   private ArrayList<ViewportSelectorCommandListener> viewportSelectorCommandListenersToRegister = new ArrayList<>();
   private ArrayList<KeyPointsChangedListener> keyPointsChangedListenersToRegister = new ArrayList<>();

   public StandardAllCommandsExecutor()
   {
   }

   public void setup(SimulationConstructionSet simulationConstructionSet, StandardSimulationGUI standardSimulationGUI, YoBuffer dataBuffer)
   {
      if (this.simulationConstructionSet != null)
         throw new RuntimeException("this.simulationConstructionSet != null");
      if (this.standardSimulationGUI != null)
         throw new RuntimeException("this.standardSimulationGUI != null");
      if (this.dataBuffer != null)
         throw new RuntimeException("this.dataBuffer != null");

      this.simulationConstructionSet = simulationConstructionSet;
      this.standardSimulationGUI = standardSimulationGUI;
      this.dataBuffer = dataBuffer;

      for (ViewportSelectorCommandListener listener : viewportSelectorCommandListenersToRegister)
      {
         standardSimulationGUI.registerViewportSelectorCommandListener(listener);
      }

      for (KeyPointsChangedListener listener : keyPointsChangedListenersToRegister)
      {
         dataBuffer.getKeyPointsHandler().addListener(listener);
      }
   }

   @Override
   public void addCameraKey()
   {
      if (standardSimulationGUI != null)
         standardSimulationGUI.addCameraKey();
   }

   public List<Integer> getCameraKeyPoints()
   {
      return standardSimulationGUI.getCameraKeyPoints();
   }

   @Override
   public void addKeyPoint()
   {
      if (standardSimulationGUI != null)
         standardSimulationGUI.setKeyPoint();
   }

   /**
    * Gets the KeyPoints in the cropped data
    *
    * @return The current KeyPoints as an ArrayList of Integer
    */
   public List<Integer> getKeyPoints()
   {
      return standardSimulationGUI.getKeyPoints();
   }

   @Override
   public GraphArrayWindow createNewGraphWindow()
   {
      return createNewGraphWindow(null);
   }

   @Override
   public GraphArrayWindow createNewGraphWindow(String graphGroupName)
   {
      String mainFrameDeviceId = standardSimulationGUI.getFrame().getGraphicsConfiguration().getDevice().getIDstring();
      String lastCharacter = mainFrameDeviceId.substring(mainFrameDeviceId.length() - 1);
      int screenIdToDisplay = 0;
      if (StringUtils.isNumeric(lastCharacter))
      {
         screenIdToDisplay = Integer.valueOf(lastCharacter);
      }
      return createNewGraphWindow(graphGroupName, screenIdToDisplay, null, null, false);
   }

   @Override
   public GraphArrayWindow createNewGraphWindow(String graphGroupName, int screenID, Point windowLocation, Dimension windowSize, boolean maximizeWindow)
   {
      if (standardSimulationGUI != null)
         return standardSimulationGUI.createNewGraphWindow(graphGroupName, screenID, windowLocation, windowSize, maximizeWindow);

      return null;
   }

   @Override
   public GraphArrayWindow getGraphArrayWindow(String windowName)
   {
      if (standardSimulationGUI != null)
         return standardSimulationGUI.getGraphArrayWindow(windowName);

      return null;
   }

   @Override
   public ViewportWindow createNewViewportWindow()
   {
      return createNewViewportWindow(null);
   }

   @Override
   public ViewportWindow createNewViewportWindow(String viewportName)
   {
      String mainFrameDeviceId = standardSimulationGUI.getFrame().getGraphicsConfiguration().getDevice().getIDstring();
      String lastCharacter = mainFrameDeviceId.substring(mainFrameDeviceId.length() - 1);
      int screenIdToDisplay = 0;
      if (StringUtils.isNumeric(lastCharacter))
      {
         screenIdToDisplay = Integer.valueOf(lastCharacter);
      }
      return createNewViewportWindow(viewportName, screenIdToDisplay, false);
   }

   @Override
   public ViewportWindow createNewViewportWindow(String viewportName, int screenID, boolean maximizeWindow)
   {
      if (standardSimulationGUI != null)
         return standardSimulationGUI.createNewViewportWindow(viewportName, screenID, maximizeWindow);
      else
         return null;
   }

   @Override
   public ViewportWindow getViewportWindow(String windowName)
   {
      if (standardSimulationGUI != null)
         return standardSimulationGUI.getViewportWindow(windowName);

      return null;
   }

   @Override
   public YoVariableSliderWindow createNewYoVariableSliderWindow()
   {
      return createNewYoVariableSliderWindow(null);
   }

   @Override
   public YoVariableSliderWindow createNewYoVariableSliderWindow(String viewportName)
   {
      String mainFrameDeviceId = standardSimulationGUI.getFrame().getGraphicsConfiguration().getDevice().getIDstring();
      String lastCharacter = mainFrameDeviceId.substring(mainFrameDeviceId.length() - 1);
      int screenIdToDisplay = 0;
      if (StringUtils.isNumeric(lastCharacter))
      {
         screenIdToDisplay = Integer.valueOf(lastCharacter);
      }
      return createNewYoVariableSliderWindow(viewportName, screenIdToDisplay, false);
   }

   @Override
   public YoVariableSliderWindow createNewYoVariableSliderWindow(String viewportName, int screenID, boolean maximizeWindow)
   {
      if (standardSimulationGUI != null)
         return standardSimulationGUI.createNewParameterSliderWindow(viewportName, screenID, maximizeWindow);
      else
         return null;
   }

   @Override
   public YoVariableSliderWindow getParameterSliderWindow(String windowName)
   {
      throw new RuntimeException("method not implemented");

   }

   @Override
   public void cropBuffer()
   {
      dataBuffer.cropBuffer();
      if (standardSimulationGUI != null)
         standardSimulationGUI.zoomFullView();
   }

   @Override
   public void packBuffer()
   {
      dataBuffer.shiftBuffer();
      if (standardSimulationGUI != null)
         standardSimulationGUI.updateGraphs();
   }

   @Override
   public void cutBuffer()
   {
      dataBuffer.cutBuffer();
      if (standardSimulationGUI != null)
         standardSimulationGUI.zoomFullView();
   }

   @Override
   public void thinBuffer(int keepEveryNthPoint)
   {
      dataBuffer.thinData(keepEveryNthPoint);
      if (standardSimulationGUI != null)
         standardSimulationGUI.zoomFullView();
   }

   @Override
   public void gotoInPoint()
   {
      if (standardSimulationGUI != null)
         standardSimulationGUI.gotoInPoint();
   }

   @Override
   public void gotoOutPoint()
   {
      if (standardSimulationGUI != null)
         standardSimulationGUI.gotoOutPoint();
   }

   @Override
   public void nextCameraKey()
   {
      if (standardSimulationGUI != null)
         standardSimulationGUI.nextCameraKey();
   }

   @Override
   public void play()
   {
      simulationConstructionSet.play();
   }

   @Override
   public void previousCameraKey()
   {
      if (standardSimulationGUI != null)
         standardSimulationGUI.previousCameraKey();
   }

   @Override
   public void removeCameraKey()
   {
      if (standardSimulationGUI != null)
         standardSimulationGUI.removeCameraKey();
   }

   @Override
   public void setInPoint()
   {
      if (standardSimulationGUI != null)
         standardSimulationGUI.setInPoint();
      else
         dataBuffer.setInPoint();
   }

   @Override
   public void setOutPoint()
   {
      if (standardSimulationGUI != null)
         standardSimulationGUI.setOutPoint();
      else
         dataBuffer.setOutPoint();
   }

   public void setInOutPointFullBuffer()
   {
      if (standardSimulationGUI != null)
         standardSimulationGUI.setInOutPointFullBuffer();
      else
         dataBuffer.setInOutPointFullBuffer();
   }

   @Override
   public void simulate()
   {
      simulationConstructionSet.simulate();
   }

   @Override
   public boolean isSimulating()
   {
      return simulationConstructionSet.isSimulating();
   }

   @Override
   public void stepBackward()
   {
      if (standardSimulationGUI != null)
         standardSimulationGUI.stepBackward();
      else
      {
         dataBuffer.tickAndReadFromBuffer(-1);
      }
   }

   @Override
   public void stepForward()
   {
      if (standardSimulationGUI != null)
         standardSimulationGUI.stepForward();
      else
      {
         dataBuffer.tickAndReadFromBuffer(1);
      }
   }

   @Override
   public void stop()
   {
      simulationConstructionSet.stop();
   }

   @Override
   public void toggleCameraKeyMode()
   {
      if (standardSimulationGUI != null)
         standardSimulationGUI.toggleCameraKeyMode();
   }

   @Override
   public boolean areKeyPointsEnabled()
   {
      return dataBuffer.getKeyPointsHandler().areKeyPointsEnabled();
   }

   @Override
   public void toggleKeyPoints()
   {
      dataBuffer.getKeyPointsHandler().toggleKeyPoints();
   }

   @Override
   public void addListener(KeyPointsChangedListener listener)
   {
      if (dataBuffer != null)
         dataBuffer.getKeyPointsHandler().addListener(listener);
      else
      {
         keyPointsChangedListenersToRegister.add(listener);
      }
   }

   @Override
   public void selectViewport(String viewportName)
   {
      if (standardSimulationGUI != null)
         standardSimulationGUI.selectViewport(viewportName);
   }

   @Override
   public void hideViewport()
   {
      if (standardSimulationGUI != null)
         EventDispatchThreadHelper.invokeLater(new Runnable()
         {
            @Override
            public void run()
            {
               standardSimulationGUI.hideViewport();
            }
         });
   }

   @Override
   public void showViewport()
   {
      if (standardSimulationGUI != null)
         EventDispatchThreadHelper.invokeLater(new Runnable()
         {
            @Override
            public void run()
            {
               standardSimulationGUI.showViewport();
            }
         });
   }

   @Override
   public boolean isViewportHidden()
   {
      if (standardSimulationGUI != null)
         return standardSimulationGUI.isViewportHidden();
      return true;
   }

   @Override
   public void registerViewportSelectorCommandListener(ViewportSelectorCommandListener commandListener)
   {
      if (standardSimulationGUI != null)
         standardSimulationGUI.registerViewportSelectorCommandListener(commandListener);
      else
      {
         viewportSelectorCommandListenersToRegister.add(commandListener);
      }
   }

   @Override
   public void zoomIn()
   {
      if (standardSimulationGUI != null)
         standardSimulationGUI.zoomIn();
   }

   @Override
   public void zoomOut()
   {
      if (standardSimulationGUI != null)
         standardSimulationGUI.zoomOut();
   }

   @Override
   public void selectGUIConfigFromFile(String fullPath)
   {
      GUIConfigurationSaveAndLoad defaultSaveAndLoad = new GUIConfigurationSaveAndLoad(simulationConstructionSet, standardSimulationGUI);
      defaultSaveAndLoad.loadGUIConfiguration(fullPath);
   }

   @Override
   public void setPlaybackRealTimeRate(double realtimeRate)
   {
      simulationConstructionSet.setPlaybackRealTimeRate(realtimeRate);
   }

   @Override
   public double getPlaybackRealTimeRate()
   {
      return simulationConstructionSet.getPlaybackRealTimeRate();
   }

   @Override
   public int getInPoint()
   {
      return simulationConstructionSet.getInPoint();
   }

   @Override
   public void setCurrentIndex(int index)
   {
      simulationConstructionSet.setCurrentIndex(index);
   }

   @Override
   public boolean tickAndReadFromBuffer(int ticks)
   {
      return simulationConstructionSet.tickAndReadFromBuffer(ticks);
   }

   public boolean tickButDoNotNotifySimulationRewoundListeners(int ticks)
   {
      return simulationConstructionSet.tickButDoNotNotifySimulationRewoundListeners(ticks);
   }

   @Override
   public int getCurrentIndex()
   {
      return simulationConstructionSet.getCurrentIndex();
   }

   @Override
   public boolean isIndexBetweenBounds(int indexToCheck)
   {
      return simulationConstructionSet.isIndexBetweenBounds(indexToCheck);
   }

   @Override
   public int getOutPoint()
   {
      return simulationConstructionSet.getOutPoint();
   }

   @Override
   public int getBufferSize()
   {
      return simulationConstructionSet.getBufferSize();
   }

   @Override
   public void exportSnapshot(File snapshotFile)
   {
      simulationConstructionSet.exportSnapshot(snapshotFile);
   }

   @Override
   public void enableGUIComponents()
   {
      simulationConstructionSet.enableGUIComponents();
   }

   @Override
   public void disableGUIComponents()
   {
      simulationConstructionSet.disableGUIComponents();
   }

   private boolean alreadyStartedClosing = false;

   @Override
   public void closeAndDispose()
   {
      if (alreadyStartedClosing)
         return;

      alreadyStartedClosing = true;

      standardSimulationGUI = null;
      simulationConstructionSet = null;
      dataBuffer = null;

      if (viewportSelectorCommandListenersToRegister != null)
      {
         for (ViewportSelectorCommandListener viewportSelectorCommandListener : viewportSelectorCommandListenersToRegister)
         {
            viewportSelectorCommandListener.closeAndDispose();
         }
         viewportSelectorCommandListenersToRegister.clear();
      }

      if (keyPointsChangedListenersToRegister != null)
      {
         keyPointsChangedListenersToRegister.clear();
      }

      viewportSelectorCommandListenersToRegister = null;
      keyPointsChangedListenersToRegister = null;
   }
}
