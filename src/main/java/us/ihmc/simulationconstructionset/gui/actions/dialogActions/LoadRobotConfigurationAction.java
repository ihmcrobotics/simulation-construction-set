package us.ihmc.simulationconstructionset.gui.actions.dialogActions;

import java.awt.event.ActionEvent;
import java.io.File;

import javax.swing.AbstractAction;
import javax.swing.Action;

import us.ihmc.simulationconstructionset.gui.dialogConstructors.LoadRobotConfigurationDialogConstructor;

public class LoadRobotConfigurationAction extends AbstractAction
{
   private static final long serialVersionUID = 5813345490164040993L;

   private LoadRobotConfigurationDialogConstructor constructor;

   public LoadRobotConfigurationAction(LoadRobotConfigurationDialogConstructor constructor)
   {
      super("Load Robot Configuration...");
      this.constructor = constructor;

      putValue(Action.LONG_DESCRIPTION, "Load Robot Configuration");
      putValue(Action.SHORT_DESCRIPTION, "load robot config");
   }

   public void setCurrentDirectory(File directory)
   {
      constructor.setCurrentDirectory(directory);
   }

   public void setCurrentDirectory(String directory)
   {
      constructor.setCurrentDirectory(directory);
   }

   @Override
   public void actionPerformed(ActionEvent e)
   {
      constructor.constructDialog();
   }

}
