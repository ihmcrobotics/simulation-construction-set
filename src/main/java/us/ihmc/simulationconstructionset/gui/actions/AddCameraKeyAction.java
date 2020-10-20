package us.ihmc.simulationconstructionset.gui.actions;

import java.awt.event.ActionEvent;
import java.awt.event.KeyEvent;

import javax.swing.AbstractAction;
import javax.swing.Action;

import us.ihmc.simulationconstructionset.commands.AddCameraKeyCommandExecutor;

public class AddCameraKeyAction extends AbstractAction
{
   private static final long serialVersionUID = -5162293334622550111L;
   private final AddCameraKeyCommandExecutor executor;

   public AddCameraKeyAction(AddCameraKeyCommandExecutor listener)
   {
      super("Add Camera Key");
      executor = listener;

      putValue(Action.MNEMONIC_KEY, new Integer(KeyEvent.VK_A));
      putValue(Action.LONG_DESCRIPTION,
               "Adds a camera key so that the camera will automatically transition to the position it is in and at the time indicated on the graph when this button is pressed.");
      putValue(Action.SHORT_DESCRIPTION, "Adds camera key.");
   }

   @Override
   public void actionPerformed(ActionEvent actionEvent)
   {
      executor.addCameraKey();
   }
}
