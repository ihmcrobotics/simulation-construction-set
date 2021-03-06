package us.ihmc.simulationconstructionset.gui.tools;

import java.awt.Dimension;
import java.util.ArrayList;
import java.util.List;

import javax.swing.JFrame;
import javax.swing.JPanel;
import javax.swing.JScrollPane;
import javax.swing.JSplitPane;

import us.ihmc.graphicsDescription.plotting.artifact.Artifact;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.graphicsDescription.yoGraphics.plotting.ArtifactList;
import us.ihmc.graphicsDescription.yoGraphics.plotting.YoArtifactPosition;
import us.ihmc.plotting.PlotterShowHideMenu;
import us.ihmc.simulationconstructionset.SimulationConstructionSet;
import us.ihmc.simulationconstructionset.gui.SimulationOverheadPlotter;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoVariable;

public class SimulationOverheadPlotterFactory
{
   private static final boolean TRACK_YAW = false;

   private SimulationConstructionSet simulationConstructionSet;
   private List<YoGraphicsListRegistry> yoGraphicsListRegistries = new ArrayList<>();

   private boolean createInSeperateWindow = false;
   private boolean showOnStart = true;
   private boolean createLegend = true;
   private boolean createShowHideMenu = true;
   private String variableNameToTrack = null;
   private String plotterName = "Plotter";

   public SimulationOverheadPlotter createOverheadPlotter()
   {
      if (simulationConstructionSet == null)
      {
         throw new RuntimeException("SimulationConstructionSet not set");
      }

      SimulationOverheadPlotter simulationOverheadPlotter = new SimulationOverheadPlotter(createLegend, createShowHideMenu);
      simulationOverheadPlotter.setDrawHistory(false);
      simulationOverheadPlotter.setXVariableToTrack(null);
      simulationOverheadPlotter.setYVariableToTrack(null);
      simulationConstructionSet.attachPlaybackListener(simulationOverheadPlotter);

      JPanel plotterPanel = simulationOverheadPlotter.getJPanel();

      JScrollPane legendScrollPane = null;
      if (createLegend)
      {
         JPanel plotterKeyJPanel = simulationOverheadPlotter.getJPanelKey();
         legendScrollPane = new JScrollPane(plotterKeyJPanel);
      }

      if (createInSeperateWindow)
      {
         JFrame overheadWindow = new JFrame(plotterName);
         overheadWindow.setSize(new Dimension(1000, 1000));
         JSplitPane splitPane = new JSplitPane(JSplitPane.HORIZONTAL_SPLIT);
         overheadWindow.add(splitPane);

         splitPane.add(plotterPanel);

         if (createLegend)
         {
            splitPane.add(legendScrollPane);
         }

         splitPane.setResizeWeight(1.0);

         for (int i = 0; i < yoGraphicsListRegistries.size(); i++)
         {
            YoGraphicsListRegistry yoGraphicsListRegistry = yoGraphicsListRegistries.get(i);
            yoGraphicsListRegistry.addArtifactListsToPlotter(simulationOverheadPlotter.getPlotter());
            ArrayList<ArtifactList> buffer = new ArrayList<>();
            yoGraphicsListRegistry.getRegisteredArtifactLists(buffer);

            if (variableNameToTrack != null && !variableNameToTrack.isEmpty())
            {
               for (ArtifactList artifactList : buffer)
               {
                  for (Artifact artifact : artifactList.getArtifacts())
                  {
                     if (artifact.getID().equals(variableNameToTrack))
                     {
                        simulationOverheadPlotter.setXVariableToTrack(((YoArtifactPosition) artifact).getYoX());
                        simulationOverheadPlotter.setYVariableToTrack(((YoArtifactPosition) artifact).getYoY());
                     }
                  }
               }
            }
         }

         overheadWindow.setVisible(showOnStart);
      }
      else
      {
         simulationConstructionSet.addExtraJpanel(plotterPanel, plotterName, showOnStart);

         if (createLegend)
         {
            simulationConstructionSet.addExtraJpanel(legendScrollPane, "Plotter Legend", false);
         }

         if (createShowHideMenu)
         {
            JScrollPane showHideMenuScrollPanel = new JScrollPane(simulationOverheadPlotter.getShowHideMenuPanel());
            showHideMenuScrollPanel.getVerticalScrollBar().setUnitIncrement(16);
            simulationConstructionSet.addExtraJpanel(showHideMenuScrollPanel, PlotterShowHideMenu.getPanelName(), false);
         }

         for (int i = 0; i < yoGraphicsListRegistries.size(); i++)
         {
            YoGraphicsListRegistry yoGraphicsListRegistry = yoGraphicsListRegistries.get(i);
            if (yoGraphicsListRegistry == null)
               continue;

            yoGraphicsListRegistry.addArtifactListsToPlotter(simulationOverheadPlotter.getPlotter());
         }

         if (variableNameToTrack != null && !variableNameToTrack.isEmpty())
         {
            YoVariable trackingVariable;
            if ((trackingVariable = simulationConstructionSet.findVariable(variableNameToTrack + "X")) != null && trackingVariable instanceof YoDouble)
            {
               simulationOverheadPlotter.setXVariableToTrack((YoDouble) trackingVariable);
            }
            if ((trackingVariable = simulationConstructionSet.findVariable(variableNameToTrack + "Y")) != null && trackingVariable instanceof YoDouble)
            {
               simulationOverheadPlotter.setYVariableToTrack((YoDouble) trackingVariable);
            }
            if (TRACK_YAW)
            {
               if ((trackingVariable = simulationConstructionSet.findVariable(variableNameToTrack + "Yaw")) != null && trackingVariable instanceof YoDouble)
               {
                  simulationOverheadPlotter.setYawVariableToTrack((YoDouble) trackingVariable);
               }
            }
         }
      }

      return simulationOverheadPlotter;
   }

   public void setSimulationConstructionSet(SimulationConstructionSet simulationConstructionSet)
   {
      this.simulationConstructionSet = simulationConstructionSet;
   }

   public void addYoGraphicsListRegistries(YoGraphicsListRegistry yoGraphicsListRegistry)
   {
      yoGraphicsListRegistries.add(yoGraphicsListRegistry);
   }

   public void setCreateInSeperateWindow(boolean createInSeperateWindow)
   {
      this.createInSeperateWindow = createInSeperateWindow;
   }

   public void setPlotterName(String plotterName)
   {
      this.plotterName = plotterName;
   }

   public void setShowOnStart(boolean showOnStart)
   {
      this.showOnStart = showOnStart;
   }

   public void setVariableNameToTrack(String variableNameToTrack)
   {
      this.variableNameToTrack = variableNameToTrack;
   }

   public void setCreateLegend(boolean createLegend)
   {
      this.createLegend = createLegend;
   }

   public void setCreateShowHideMenu(boolean createShowHideMenu)
   {
      this.createShowHideMenu = createShowHideMenu;
   }
}
