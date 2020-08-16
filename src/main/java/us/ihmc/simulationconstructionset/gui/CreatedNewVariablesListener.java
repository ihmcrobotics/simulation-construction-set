package us.ihmc.simulationconstructionset.gui;

import us.ihmc.yoVariables.registry.YoVariableList;
import us.ihmc.yoVariables.variable.YoVariable;

public interface CreatedNewVariablesListener
{
   public abstract void createdNewVariables(YoVariableList newVariables);

   public abstract void createdNewVariable(YoVariable variable);
}
