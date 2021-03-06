package us.ihmc.simulationconstructionset.gui.actions.configActions;

import java.awt.event.ActionEvent;

import javax.swing.AbstractAction;
import javax.swing.Action;

import us.ihmc.simulationconstructionset.gui.config.GraphGroupSelector;

public class SelectGraphGroupAction extends AbstractAction
{
   private static final long serialVersionUID = -5534400254284863190L;
   private String name;
   private GraphGroupSelector selector;

   public SelectGraphGroupAction(GraphGroupSelector selector, String name)
   {
      super(name);

      this.selector = selector;
      this.name = name;

      // this.putValue(Action.MNEMONIC_KEY, new Integer(KeyEvent.VK_E));
      putValue(Action.LONG_DESCRIPTION, "Long Description");
      putValue(Action.SHORT_DESCRIPTION, "Short Description");
   }

   @Override
   public void actionPerformed(ActionEvent e)
   {
      selector.selectGraphGroup(name);
   }
}
