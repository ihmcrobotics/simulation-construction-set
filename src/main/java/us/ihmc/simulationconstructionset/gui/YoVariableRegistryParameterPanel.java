package us.ihmc.simulationconstructionset.gui;

import java.util.ArrayList;
import java.util.List;

import us.ihmc.graphicsDescription.graphInterfaces.SelectedVariableHolder;
import us.ihmc.simulationconstructionset.gui.yoVariableSearch.YoVariablePanel;
import us.ihmc.simulationconstructionset.gui.yoVariableSearch.YoVariablePanelJPopupMenu;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoVariable;

public class YoVariableRegistryParameterPanel extends YoVariablePanel
{
   private static final long serialVersionUID = -4138864557620938578L;
   private final List<YoVariable> parameters = new ArrayList<>();

   public YoVariableRegistryParameterPanel(YoRegistry registry, SelectedVariableHolder holder, YoVariablePanelJPopupMenu varPanelJPopupMenu)
   {
      super(registry.getName(), holder, varPanelJPopupMenu);

      List<YoVariable> registryVariables = registry.collectSubtreeVariables();
      for (YoVariable var : registryVariables)
      {
         if (var.isParameter())
         {
            parameters.add(var);
         }
      }
      clearAndSetUpTextFields();
   }

   @Override
   protected YoVariable getYoVariable(int index)
   {
      return parameters.get(index);
   }

   @Override
   protected List<YoVariable> getAllYoVariablesCopy()
   {
      return new ArrayList<>(parameters);
   }

   @Override
   protected int getNumberOfYoVariables()
   {
      return parameters.size();
   }

   @Override
   public YoVariable getYoVariable(String name)
   {
      for (YoVariable var : parameters)
      {
         if (var.getName().equals(name))
         {
            return var;
         }
      }
      return null;
   }
}
