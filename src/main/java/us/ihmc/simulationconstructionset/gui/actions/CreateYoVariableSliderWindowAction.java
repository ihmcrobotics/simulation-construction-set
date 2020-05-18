package us.ihmc.simulationconstructionset.gui.actions;

import java.awt.event.ActionEvent;
import java.awt.event.KeyEvent;

import javax.swing.AbstractAction;
import javax.swing.Action;

import us.ihmc.simulationconstructionset.commands.CreateNewYoVariableSliderWindowCommandExecutor;

/**
 * Created by Peter on 8/29/2017.
 */

public class CreateYoVariableSliderWindowAction extends AbstractAction
{
   private static final long serialVersionUID = 5063019587274426802L;
   private CreateNewYoVariableSliderWindowCommandExecutor executor;

   public CreateYoVariableSliderWindowAction(CreateNewYoVariableSliderWindowCommandExecutor executor)
   {
      super("New Slider Window");
      this.executor = executor;

      putValue(Action.MNEMONIC_KEY, new Integer(KeyEvent.VK_S));
      putValue(Action.LONG_DESCRIPTION, "Creates a new YoVariable Slider window for change yoVariables");
      putValue(Action.SHORT_DESCRIPTION, "New YoVariable Slider Window");
   }

   @Override
   public void actionPerformed(ActionEvent actionEvent)
   {
      executor.createNewYoVariableSliderWindow();
   }
}
