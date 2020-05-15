package us.ihmc.simulationconstructionset.dataBuffer;

import java.util.List;

import us.ihmc.simulationconstructionset.gui.config.VarGroup;
import us.ihmc.simulationconstructionset.gui.config.VarGroupList;
import us.ihmc.yoVariables.dataBuffer.DataBuffer;
import us.ihmc.yoVariables.variable.YoVariable;

public class DataBufferTools
{
   public static List<YoVariable<?>> getVarsFromGroup(DataBuffer dataBuffer, String varGroupName, VarGroupList varGroupList)
   {
      if (varGroupName.equals("all"))
      {
         return dataBuffer.getVariables();
      }

      VarGroup varGroup = varGroupList.getVarGroup(varGroupName);
      String[] varNames = varGroup.getVars();
      String[] regularExpressions = varGroup.getRegularExpressions();

      return dataBuffer.getVars(varNames, regularExpressions);
   }
}
