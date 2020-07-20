package us.ihmc.simulationconstructionset.gui;

import java.util.ArrayList;

import javax.swing.JFrame;
import javax.swing.SwingUtilities;

import us.ihmc.simulationconstructionset.gui.dialogs.VarPropertiesDialog;
import us.ihmc.yoVariables.buffer.YoBuffer;
import us.ihmc.yoVariables.buffer.YoBufferVariableEntry;
import us.ihmc.yoVariables.variable.YoVariable;

public class YoVariableDoubleClickListener implements DoubleClickListener
{
   private YoBuffer dataBuffer;
   private JFrame parentFrame;

   public YoVariableDoubleClickListener(YoBuffer dataBuffer, JFrame frame)
   {
      this.dataBuffer = dataBuffer;
      parentFrame = frame;

   }

   @Override
   public void doubleClicked(YoVariable v)
   {
      YoBufferVariableEntry entry = dataBuffer.getEntry(v);

      if (entry == null)
         return;

      ArrayList<YoBufferVariableEntry> entries = new ArrayList<>();
      entries.add(entry);

      SwingUtilities.invokeLater(() -> new VarPropertiesDialog(parentFrame, entries));
   }

}
