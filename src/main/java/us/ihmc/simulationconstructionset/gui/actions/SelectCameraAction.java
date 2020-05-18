package us.ihmc.simulationconstructionset.gui.actions;

import java.awt.event.ActionEvent;

import javax.swing.AbstractAction;
import javax.swing.Action;

import us.ihmc.simulationconstructionset.gui.config.CameraSelector;

public class SelectCameraAction extends AbstractAction
{
   private static final long serialVersionUID = -6442315727953394627L;
   private String name;
   private CameraSelector cameraSelector;

   public SelectCameraAction(CameraSelector selector, String name)
   {
      super(name);

      cameraSelector = selector;
      this.name = name;

      // this.putValue(Action.MNEMONIC_KEY, new Integer(KeyEvent.VK_E));
      putValue(Action.LONG_DESCRIPTION, "Long Description");
      putValue(Action.SHORT_DESCRIPTION, "Short Description");
   }

   @Override
   public void actionPerformed(ActionEvent e)
   {
      cameraSelector.selectCamera(name);
   }
}
