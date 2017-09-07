package us.ihmc.simulationconstructionset.gui.actions.configActions;

import java.awt.event.ActionEvent;

import javax.swing.AbstractAction;
import javax.swing.Action;

import us.ihmc.simulationconstructionset.commands.ViewportSelectorCommandExecutor;


public class SelectViewportAction extends AbstractAction
{
   private static final long serialVersionUID = 6565081288826811421L;
   private String name;

   private ViewportSelectorCommandExecutor selector;

   public SelectViewportAction(ViewportSelectorCommandExecutor selector, String name)
   {
      super(name);

      this.selector = selector;
      this.name = name;

      // this.putValue(Action.MNEMONIC_KEY, new Integer(KeyEvent.VK_E));
      this.putValue(Action.LONG_DESCRIPTION, "Long Description");
      this.putValue(Action.SHORT_DESCRIPTION, "Short Description");
   }

   @Override
   public void actionPerformed(ActionEvent e)
   {
      selector.selectViewport(name);
   }
}
