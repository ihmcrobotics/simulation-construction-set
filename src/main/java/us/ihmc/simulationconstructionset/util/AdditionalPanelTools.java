package us.ihmc.simulationconstructionset.util;

import java.util.function.Predicate;

import javax.swing.JLabel;

import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.simulationconstructionset.SimulationConstructionSet;
import us.ihmc.simulationconstructionset.gui.StandardSimulationGUI;
import us.ihmc.yoVariables.variable.YoVariable;

public class AdditionalPanelTools
{
   /**
    * Adds a jpanel to the provided SCS instance that is available under Viewport -> Extra Panels.
    * It will show the reference frame that the selected YoVariable represents if that variable
    * satisfies the provided filter and the provided frame index map contains a frame mapping for
    * the long value of the variable. This also adds a label to the variable search box that will
    * appear and show the frame if the selected variable satisfies the filter.
    *
    * @param scs
    *           the instance of SCS that the panel will be added to.
    * @param frameIndexMap
    *           provides a map from frame index to reference frame.
    * @param variableFilter
    *           indicates which variables should be representing a frame.
    */
   public static void setupFrameView(SimulationConstructionSet scs, FrameMap frameIndexMap, Predicate<YoVariable<?>> variableFilter)
   {
      JLabel frameNameLabel = new JLabel();
      scs.addExtraJpanel(frameNameLabel, "Frame Information", false);
      updateFrameLabel(frameIndexMap, variableFilter, frameNameLabel, null);
      StandardSimulationGUI gui = scs.getStandardSimulationGUI();
      gui.addSelectedVariableChangedListener(e -> updateFrameLabel(frameIndexMap, variableFilter, frameNameLabel, gui.getSelectedVariable()));
      gui.setFrameMap(frameIndexMap, variableFilter);
   }

   private static void updateFrameLabel(FrameMap frameIndexMap, Predicate<YoVariable<?>> variableFilter, JLabel frameNameLabel, YoVariable<?> variable)
   {
      if (variable == null)
      {
         frameNameLabel.setText("No variable selected.");
         return;
      }

      String text = "Not a frame.";
      if (variableFilter.test(variable))
      {
         ReferenceFrame frame = frameIndexMap.getReferenceFrame(variable.getValueAsLongBits());
         if (frame != null)
         {
            text = frame.getName();
         }
         else
         {
            text = "Unknown Frame";
         }
      }
      frameNameLabel.setText("<html>" + variable.getName() + "<br/>" + text);
   }

   public static interface FrameMap
   {
      ReferenceFrame getReferenceFrame(long frameIndex);
   }
}
