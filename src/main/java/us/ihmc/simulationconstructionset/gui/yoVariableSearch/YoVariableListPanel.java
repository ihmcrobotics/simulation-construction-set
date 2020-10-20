package us.ihmc.simulationconstructionset.gui.yoVariableSearch;

import java.awt.Component;
import java.util.ArrayList;
import java.util.List;

import us.ihmc.graphicsDescription.graphInterfaces.SelectedVariableHolder;
import us.ihmc.simulationconstructionset.gui.EventDispatchThreadHelper;
import us.ihmc.yoVariables.registry.YoVariableList;
import us.ihmc.yoVariables.variable.YoVariable;

public class YoVariableListPanel extends YoVariablePanel
{
   private static final long serialVersionUID = -995471860623897960L;
   private YoVariableList varList;

   public YoVariableListPanel(String name, SelectedVariableHolder holder, YoVariablePanelJPopupMenu varPanelJPopupMenu, YoVariableSearchPanel searchPanel)
   {
      this(new YoVariableList(name), holder, varPanelJPopupMenu, searchPanel);
   }

   public YoVariableListPanel(YoVariableList list, SelectedVariableHolder holder, YoVariablePanelJPopupMenu varPanelJPopupMenu,
                              YoVariableSearchPanel searchPanel)
   {
      super(list.getName(), holder, varPanelJPopupMenu, searchPanel);
      varList = list;
      clearAndSetUpTextFields();
   }

   public YoVariableListPanel(String name, SelectedVariableHolder holder, YoVariablePanelJPopupMenu varPanelJPopupMenu)
   {
      this(new YoVariableList(name), holder, varPanelJPopupMenu, null);
   }

   public YoVariableListPanel(YoVariableList list, SelectedVariableHolder holder, YoVariablePanelJPopupMenu varPanelJPopupMenu)
   {
      this(list, holder, varPanelJPopupMenu, null);
   }

   @Override
   protected int getNumberOfYoVariables()
   {
      if (varList == null)
         return 0;

      return varList.size();
   }

   @Override
   protected YoVariable getYoVariable(int index)
   {
      return varList.get(index);
   }

   @Override
   protected List<YoVariable> getAllYoVariablesCopy()
   {
      return new ArrayList<>(varList.getVariables());
   }

   @Override
   public YoVariable getYoVariable(String name)
   {
      return varList.findVariable(name);
   }

   public void addVariable(final YoVariable v)
   {
      if (v != null)
      {
         varList.add(v);

         EventDispatchThreadHelper.invokeAndWait(new Runnable()
         {
            @Override
            public void run()
            {
               addTextFieldForVariable(v);
            }
         });
      }
   }

   public void removeVariable(YoVariable v)
   {
      int indexOfVariableValueToRemove = varList.indexOf(v);
      if ((indexOfVariableValueToRemove >= 0) && (indexOfVariableValueToRemove < getYoVariableSpinners().size()))
      {
         System.out.println("removing var");
         Component field = getYoVariableSpinners().remove(indexOfVariableValueToRemove);
         this.remove(field);
      }

      varList.remove(v);
      invalidate();
      revalidate();
      updateUI();
      this.repaint();
   }

   public void removeAllVariables()
   {
      varList.clear();
      getYoVariableSpinners().clear();
      removeAll();

      //    this.updateUI();
   }

   @Override
   public String toString()
   {
      StringBuffer retBuffer = new StringBuffer();

      retBuffer.append(varList.toString());

      return retBuffer.toString();
   }
}