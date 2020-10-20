package us.ihmc.simulationconstructionset.gui.dialogConstructors;

import java.io.FileOutputStream;
import java.io.IOException;

import javax.swing.JOptionPane;

import us.ihmc.simulationconstructionset.SimulationConstructionSet;
import us.ihmc.yoVariables.parameters.XmlParameterWriter;
import us.ihmc.yoVariables.registry.YoRegistry;

public class SaveParametersGenerator implements SaveParametersConstructor
{
   private final SimulationConstructionSet scs;
   private ParameterFileChooser fileChooser;

   public SaveParametersGenerator(SimulationConstructionSet scs)
   {
      this.scs = scs;
      fileChooser = new ParameterFileChooser();
   }

   @Override
   public void constructDialog()
   {

      if (fileChooser.showDialog(scs.getJFrame(), scs.getRootRegistry(), scs.getParameterRootPath(), scs.getDefaultParameterFile(), true))
      {
         XmlParameterWriter writer = new XmlParameterWriter();
         for (YoRegistry child : fileChooser.getRegistries())
         {
            writer.addParameters(child);
         }
         try
         {
            FileOutputStream os = new FileOutputStream(fileChooser.getFile());
            writer.write(os);
            os.close();
         }
         catch (IOException e)
         {
            JOptionPane.showMessageDialog(scs.getJFrame(),
                                          "Cannot write to " + fileChooser.getFile() + "\n" + e.getMessage(),
                                          "Cannot write to file",
                                          JOptionPane.ERROR_MESSAGE);
         }
      }

   }

   @Override
   public void closeAndDispose()
   {
      fileChooser.closeAndDispose();
      fileChooser = null;
   }

}
