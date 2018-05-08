package us.ihmc.simulationconstructionset.util.simulationRunner;


import static org.junit.Assert.*;

import java.util.ArrayList;

import org.junit.Test;

import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationTest;
import us.ihmc.simulationconstructionset.util.simulationRunner.VariableDifference;
import us.ihmc.simulationconstructionset.util.simulationRunner.VariablesThatShouldMatchList;
import us.ihmc.yoVariables.registry.NameSpace;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoDouble;

public class VariablesThatShouldMatchListTest
{
	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout=300000)
   public void testOne()
   {
      YoVariableRegistry registryOne = new YoVariableRegistry("root");
      YoVariableRegistry registryTwo = new YoVariableRegistry("root");

      registryOne.getOrCreateAndAddRegistry(new NameSpace("root.one.two.three.four"));
      registryTwo.getOrCreateAndAddRegistry(new NameSpace("root.one.two.three.four"));

      ArrayList<YoDouble[]> theseShouldMatch = new ArrayList<YoDouble[]>();
      
      theseShouldMatch.add(createAndAddVariable("var1", "root.one.two.three.four", registryOne, registryTwo));
      theseShouldMatch.add(createAndAddVariable("var2", "root.one.two.three.four", registryOne, registryTwo));
      theseShouldMatch.add(createAndAddVariable("var3", "root.one.two.three", registryOne, registryTwo));
      theseShouldMatch.add(createAndAddVariable("var4", "root.one.two", registryOne, registryTwo));
      theseShouldMatch.add(createAndAddVariable("var5", "root.one", registryOne, registryTwo));
      theseShouldMatch.add(createAndAddVariable("var6", "root", registryOne, registryTwo));

      ArrayList<YoDouble[]> theseDontNeedToMatch = new ArrayList<YoDouble[]>();
      theseDontNeedToMatch.add(createAndAddVariable("var1_exception1", "root.one.two.three.four", registryOne, registryTwo));
      theseDontNeedToMatch.add(createAndAddVariable("var2_exception2", "root.one.two.three.four", registryOne, registryTwo));
      theseDontNeedToMatch.add(createAndAddVariable("exception3", "root.one", registryOne, registryTwo));

      ArrayList<String> exceptions = new ArrayList<String>();
      exceptions.add("exception1");
      exceptions.add("exception2");
      exceptions.add("exception3");
      
      VariablesThatShouldMatchList variablesThatShouldMatchList = new VariablesThatShouldMatchList(registryOne, registryTwo, exceptions);
      
      ArrayList<VariableDifference> variableDifferences = new ArrayList<VariableDifference>();
      double maxDifferenceAllowed = Double.MIN_VALUE;
      boolean checkForPercentDifference = false;
      boolean doValuesMatch = variablesThatShouldMatchList.doVariableValuesMatch(variableDifferences, 0.0, maxDifferenceAllowed, checkForPercentDifference);
      
      assertTrue(doValuesMatch);
      
      registryOne.getVariable("var1").setValueFromDouble(0.123);
      doValuesMatch = variablesThatShouldMatchList.doVariableValuesMatch(variableDifferences, 0.0, maxDifferenceAllowed, checkForPercentDifference);
      assertEquals(1, variableDifferences.size());
      assertEquals(variableDifferences.get(0).getVariableOne().getName(), "var1");
      assertFalse(doValuesMatch);
      
      variableDifferences.clear();
      registryTwo.getVariable("var1").setValueFromDouble(0.123);
      doValuesMatch = variablesThatShouldMatchList.doVariableValuesMatch(variableDifferences, 0.0, maxDifferenceAllowed, checkForPercentDifference);
      assertTrue(doValuesMatch);
      
      registryTwo.getVariable("var1_exception1").setValueFromDouble(0.123);
      registryTwo.getVariable("var2_exception2").setValueFromDouble(0.456);
      registryTwo.getVariable("exception3").setValueFromDouble(0.789);

      doValuesMatch = variablesThatShouldMatchList.doVariableValuesMatch(variableDifferences, 0.0, maxDifferenceAllowed, checkForPercentDifference);
      assertTrue(doValuesMatch);
      
      registryOne.getVariable("var2").setValueFromDouble(11.23);
      registryTwo.getVariable("var4").setValueFromDouble(14.5);
      registryTwo.getVariable("var6").setValueFromDouble(16.7);
      doValuesMatch = variablesThatShouldMatchList.doVariableValuesMatch(variableDifferences, 0.0, maxDifferenceAllowed, checkForPercentDifference);
      assertEquals(3, variableDifferences.size());
      assertEquals(variableDifferences.get(0).getVariableOne().getName(), "var6");
      assertEquals(variableDifferences.get(1).getVariableOne().getName(), "var4");
      assertEquals(variableDifferences.get(2).getVariableOne().getName(), "var2");
      assertFalse(doValuesMatch);
      
      variableDifferences.clear();
      registryTwo.getVariable("var2").setValueFromDouble(11.23);
      registryOne.getVariable("var4").setValueFromDouble(14.5);
      registryOne.getVariable("var6").setValueFromDouble(16.7);
      doValuesMatch = variablesThatShouldMatchList.doVariableValuesMatch(variableDifferences, 0.0, maxDifferenceAllowed, checkForPercentDifference);
      assertEquals(0, variableDifferences.size());
      assertTrue(doValuesMatch);

      variableDifferences.clear();
      maxDifferenceAllowed = 0.11;
      checkForPercentDifference = false;
      registryOne.getVariable("var1").setValueFromDouble(111.0);
      registryTwo.getVariable("var1").setValueFromDouble(110.9);
      doValuesMatch = variablesThatShouldMatchList.doVariableValuesMatch(variableDifferences, 0.0, maxDifferenceAllowed, checkForPercentDifference);
      assertTrue(doValuesMatch);
      maxDifferenceAllowed = 0.099;
      doValuesMatch = variablesThatShouldMatchList.doVariableValuesMatch(variableDifferences, 0.0, maxDifferenceAllowed, checkForPercentDifference);
      assertFalse(doValuesMatch);
      
      variableDifferences.clear();
      maxDifferenceAllowed = 0.0099;
      checkForPercentDifference = true;
      registryOne.getVariable("var1").setValueFromDouble(10.0);
      registryTwo.getVariable("var1").setValueFromDouble(10.1);
      doValuesMatch = variablesThatShouldMatchList.doVariableValuesMatch(variableDifferences, 0.0, maxDifferenceAllowed, checkForPercentDifference);
      assertFalse(doValuesMatch);
      variableDifferences.clear();
      maxDifferenceAllowed = 0.011;
      doValuesMatch = variablesThatShouldMatchList.doVariableValuesMatch(variableDifferences, 0.0, maxDifferenceAllowed, checkForPercentDifference);
      assertTrue(doValuesMatch);
      
      variableDifferences.clear();
      
      YoDouble onlyInOne = new YoDouble("onlyInOne", registryOne);
      YoDouble onlyInTwo = new YoDouble("onlyInTwo", registryTwo);
    
      variablesThatShouldMatchList = new VariablesThatShouldMatchList(registryOne, registryTwo, exceptions);
      
      doValuesMatch = variablesThatShouldMatchList.doVariableValuesMatch(variableDifferences, 0.0, maxDifferenceAllowed, checkForPercentDifference);
      assertFalse(doValuesMatch);
      assertEquals(2, variableDifferences.size());
      assertEquals("onlyInOne", variableDifferences.get(0).getVariableOne().getName());
      assertNull(variableDifferences.get(0).getVariableTwo());
      assertEquals("onlyInTwo", variableDifferences.get(1).getVariableTwo().getName());
      assertNull(variableDifferences.get(1).getVariableOne());

   }


   private YoDouble[] createAndAddVariable(String name, String fullNameSpace, YoVariableRegistry rootRegistryOne, YoVariableRegistry rootRegistryTwo)
   {
      YoDouble[] ret = new YoDouble[2];
      ret[0] = new YoDouble(name, rootRegistryOne.getOrCreateAndAddRegistry(new NameSpace(fullNameSpace)));
      ret[1] = new YoDouble(name, rootRegistryTwo.getOrCreateAndAddRegistry(new NameSpace(fullNameSpace)));

      return ret;
   }
}
