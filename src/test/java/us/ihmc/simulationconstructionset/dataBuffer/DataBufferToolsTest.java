package us.ihmc.simulationconstructionset.dataBuffer;

import static us.ihmc.robotics.Assert.assertFalse;
import static us.ihmc.robotics.Assert.assertTrue;

import java.util.List;

import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

import us.ihmc.simulationconstructionset.gui.config.VarGroup;
import us.ihmc.simulationconstructionset.gui.config.VarGroupList;
import us.ihmc.yoVariables.buffer.YoBuffer;
import us.ihmc.yoVariables.buffer.YoBufferVariableEntry;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoVariable;

public class DataBufferToolsTest
{
   private final int testBufferSize = 100;

   private YoRegistry registry;
   private YoBuffer dataBuffer = new YoBuffer(testBufferSize);

   private YoDouble a, b, c;
   private YoBufferVariableEntry aBuffer, bBuffer, cBuffer;

   @BeforeEach
   public void setUp()
   {
      registry = new YoRegistry("testRegistry");

      a = new YoDouble("a_arm", registry);
      b = new YoDouble("b_arm", registry);
      c = new YoDouble("c_arm", registry);

      aBuffer = new YoBufferVariableEntry(a, testBufferSize);
      bBuffer = new YoBufferVariableEntry(b, testBufferSize);
      cBuffer = new YoBufferVariableEntry(c, testBufferSize);
   }

   @Test // timeout=300000
   public void testGetVarsFromGroup()
   {
      dataBuffer.addEntry(aBuffer);
      dataBuffer.addEntry(bBuffer);
      dataBuffer.addEntry(cBuffer);

      VarGroup varGroupOne = new VarGroup("varGroupOne");
      VarGroup varGroupTwo = new VarGroup("varGroupTwo");
      VarGroup varGroupThree = new VarGroup("varGroupThree");

      varGroupOne.addVar("a_arm");
      varGroupOne.addVar("c_arm");

      String[] allRegularExpressions = {".*"};
      String[] cRegularExpressions = {"c.*"};

      varGroupTwo.addRegularExpressions(allRegularExpressions);
      varGroupThree.addRegularExpressions(cRegularExpressions);

      VarGroupList varGroupList = new VarGroupList();
      varGroupList.addVarGroup(varGroupOne);
      varGroupList.addVarGroup(varGroupTwo);
      varGroupList.addVarGroup(varGroupThree);

      List<YoVariable> allVarsFromGroup = DataBufferTools.getVarsFromGroup(dataBuffer, "all", varGroupList);

      assertTrue(allVarsFromGroup.contains(a));
      assertTrue(allVarsFromGroup.contains(b));
      assertTrue(allVarsFromGroup.contains(c));

      List<YoVariable> aVarsFromGroup = DataBufferTools.getVarsFromGroup(dataBuffer, "varGroupOne", varGroupList);

      assertTrue(aVarsFromGroup.contains(a));
      assertFalse(aVarsFromGroup.contains(b));
      assertTrue(aVarsFromGroup.contains(c));

      List<YoVariable> regExpVarsFromGroup = DataBufferTools.getVarsFromGroup(dataBuffer, "varGroupTwo", varGroupList);

      assertTrue(regExpVarsFromGroup.contains(a));
      assertTrue(regExpVarsFromGroup.contains(b));
      assertTrue(regExpVarsFromGroup.contains(c));

      List<YoVariable> cRegExpVarsFromGroup = DataBufferTools.getVarsFromGroup(dataBuffer, "varGroupThree", varGroupList);

      assertFalse(cRegExpVarsFromGroup.contains(a));
      assertFalse(cRegExpVarsFromGroup.contains(b));
      assertTrue(cRegExpVarsFromGroup.contains(c));
   }
}
