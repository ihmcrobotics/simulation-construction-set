package us.ihmc.simulationconstructionset.dataBuffer;

import java.util.ArrayList;
import java.util.List;
import java.util.stream.Stream;

import us.ihmc.simulationconstructionset.gui.config.VarGroup;
import us.ihmc.simulationconstructionset.gui.config.VarGroupList;
import us.ihmc.yoVariables.dataBuffer.YoBuffer;
import us.ihmc.yoVariables.tools.YoSearchTools;
import us.ihmc.yoVariables.variable.YoVariable;

public class DataBufferTools
{
   public static List<YoVariable> getVarsFromGroup(YoBuffer dataBuffer, String varGroupName, VarGroupList varGroupList)
   {
      if (varGroupName.equals("all"))
      {
         return dataBuffer.getVariables();
      }

      VarGroup varGroup = varGroupList.getVarGroup(varGroupName);
      String[] varNames = varGroup.getVars();
      String[] regularExpressions = varGroup.getRegularExpressions();
      List<YoVariable> variables = new ArrayList<YoVariable>();
      if (varNames != null)
         Stream.of(varNames).flatMap(varName -> dataBuffer.findVariables(varName).stream()).forEach(variables::add);
      if (regularExpressions != null)
         variables.addAll(dataBuffer.filterVariables(YoSearchTools.regularExpressionFilter(regularExpressions)));
      return variables;
   }
}
