package us.ihmc.simulationconstructionset.gui;

import java.util.ArrayList;
import java.util.List;

import javax.swing.event.ChangeListener;

import us.ihmc.graphicsDescription.graphInterfaces.SelectedVariableHolder;
import us.ihmc.simulationconstructionset.gui.yoVariableSearch.YoVariablePanel;
import us.ihmc.simulationconstructionset.gui.yoVariableSearch.YoVariablePanelJPopupMenu;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoVariable;

public class YoVariableRegistryVarPanel extends YoVariablePanel
{
   private static final long serialVersionUID = -9079475583549031191L;
   private final YoRegistry registry;

   public YoVariableRegistryVarPanel(YoRegistry registry, SelectedVariableHolder holder, YoVariablePanelJPopupMenu varPanelJPopupMenu)
   {
      super(registry.getName(), holder, varPanelJPopupMenu);
      this.registry = registry;
      clearAndSetUpTextFields();
   }

   @Override
   protected YoVariable getYoVariable(int index)
   {
      return registry.getVariable(index);
   }

   @Override
   protected List<YoVariable> getAllYoVariablesCopy()
   {
      return new ArrayList<>(registry.getVariables());
   }

   @Override
   protected int getNumberOfYoVariables()
   {
      return registry.getNumberOfVariables();
   }

   @Override
   public YoVariable getYoVariable(String name)
   {
      return registry.findVariable(name);
   }

   @Override
   public void addChangeListener(ChangeListener changeListener)
   {
      throw new RuntimeException("YoRegistryVarList.addChangeListener() not yet implemented.");
   }
}
