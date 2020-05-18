package us.ihmc.simulationconstructionset.gui;

import java.util.ArrayList;

import javax.swing.JFrame;
import javax.swing.SwingUtilities;

import us.ihmc.simulationconstructionset.gui.dialogs.VarPropertiesDialog;
import us.ihmc.yoVariables.dataBuffer.DataBuffer;
import us.ihmc.yoVariables.dataBuffer.DataBufferEntry;
import us.ihmc.yoVariables.variable.YoVariable;

public class YoVariableDoubleClickListener implements DoubleClickListener
{
   private DataBuffer dataBuffer;
   private JFrame parentFrame;

   public YoVariableDoubleClickListener(DataBuffer dataBuffer, JFrame frame)
   {
      this.dataBuffer = dataBuffer;
      parentFrame = frame;

   }

   @Override
   public void doubleClicked(YoVariable<?> v)
   {
      DataBufferEntry entry = dataBuffer.getEntry(v);

      if (entry == null)
         return;

      ArrayList<DataBufferEntry> entries = new ArrayList<>();
      entries.add(entry);

      SwingUtilities.invokeLater(() -> new VarPropertiesDialog(parentFrame, entries));
   }

}
