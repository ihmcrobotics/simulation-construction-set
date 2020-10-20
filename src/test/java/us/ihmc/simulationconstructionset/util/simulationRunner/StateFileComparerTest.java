package us.ihmc.simulationconstructionset.util.simulationRunner;

import static us.ihmc.robotics.Assert.assertEquals;
import static us.ihmc.robotics.Assert.assertFalse;
import static us.ihmc.robotics.Assert.assertTrue;

import java.io.File;
import java.io.FileNotFoundException;
import java.io.PrintStream;
import java.util.ArrayList;

import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.registry.YoVariableList;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoDouble;

public class StateFileComparerTest
{

   @BeforeEach
   public void setUp() throws Exception
   {
   }

   @AfterEach
   public void tearDown() throws Exception
   {
   }

   @Test // timeout=300000
   public void testAbsoluteCompareStateFiles() throws FileNotFoundException
   {
      File fileOne = new File("fileOne.state");
      File fileTwo = new File("fileTwo.state");

      if (fileOne.exists())
         fileOne.delete();
      if (fileTwo.exists())
         fileTwo.delete();

      writeStateFileA(fileOne);
      writeStateFileA(fileTwo);
      verifyNoDifferences(fileOne, fileTwo);
      fileOne.delete();
      fileTwo.delete();

      writeStateFileB(fileOne);
      writeStateFileB(fileTwo);
      verifyNoDifferences(fileOne, fileTwo);
      fileOne.delete();
      fileTwo.delete();

      writeStateFileA(fileOne);
      writeStateFileB(fileTwo);
      verifyDifferences(fileOne, fileTwo);
      fileOne.delete();
      fileTwo.delete();
   }

   private void verifyNoDifferences(File fileOne, File fileTwo)
   {
      double maxAbsoluteDiff = 1e-7;
      ArrayList<String> exceptions = new ArrayList<>();

      ArrayList<VariableDifference> variableDifferences = StateFileComparer.absoluteCompareStateFiles(fileOne, fileTwo, maxAbsoluteDiff, exceptions);
      assertTrue(VariableDifference.allVariableDifferencesToString(variableDifferences), variableDifferences.isEmpty());
   }

   private void verifyDifferences(File fileOne, File fileTwo)
   {
      double maxAbsoluteDiff = 1e-7;
      ArrayList<String> exceptions = new ArrayList<>();

      ArrayList<VariableDifference> variableDifferences = StateFileComparer.absoluteCompareStateFiles(fileOne, fileTwo, maxAbsoluteDiff, exceptions);
      assertFalse(variableDifferences.isEmpty());
   }

   private void writeStateFileA(File file) throws FileNotFoundException
   {
      PrintStream stream = new PrintStream(file);

      stream.println("BipedalRobot.Time.gravityZ = -9.81;");
      stream.println("launchedBalls.Time.gravityZ = -9.8;");
      stream.close();
   }

   private void writeStateFileB(File file) throws FileNotFoundException
   {
      PrintStream stream = new PrintStream(file);

      stream.println("variable1 = 0.1;");
      stream.println("variable2 = 0.2;");
      stream.close();
   }

   @Test // timeout=300000
   public void testCompareVarLists()
   {
      YoRegistry registry1 = createRegistryAndFillWithVariables();
      YoRegistry registry2 = createRegistryAndFillWithVariables();

      ArrayList<String> exceptions = new ArrayList<>();
      exceptions.add(new String("exceptional"));

      ArrayList<VariableDifference> variableDifferences = StateFileComparer.compareVarLists(new YoVariableList(registry1.getName(), registry1.getVariables()),
                                                                                            new YoVariableList(registry2.getName(), registry2.getVariables()),
                                                                                            1e-7,
                                                                                            false,
                                                                                            exceptions);
      assertEquals(0, variableDifferences.size());

      ((YoDouble) registry1.findVariable("exceptionalVariable")).set(5678.0);

      variableDifferences = StateFileComparer.compareVarLists(new YoVariableList(registry1.getName(), registry1.getVariables()),
                                                              new YoVariableList(registry2.getName(), registry2.getVariables()),
                                                              1e-7,
                                                              false,
                                                              exceptions);
      assertEquals(0, variableDifferences.size());

      ((YoDouble) registry1.findVariable("variable1")).set(3.5);

      variableDifferences = StateFileComparer.compareVarLists(new YoVariableList(registry1.getName(), registry1.getVariables()),
                                                              new YoVariableList(registry2.getName(), registry2.getVariables()),
                                                              1e-7,
                                                              false,
                                                              exceptions);
      assertEquals(1, variableDifferences.size());
   }

   private YoRegistry createRegistryAndFillWithVariables()
   {
      YoRegistry root = new YoRegistry("root");
      YoRegistry registry0 = new YoRegistry("registry0");
      YoRegistry registry00 = new YoRegistry("registry00");
      YoRegistry registry01 = new YoRegistry("registry01");

      root.addChild(registry0);
      registry0.addChild(registry00);
      registry0.addChild(registry01);

      YoDouble variable1 = new YoDouble("variable1", root);
      YoDouble variable2 = new YoDouble("variable2", registry0);
      YoDouble variable3 = new YoDouble("variable3", registry00);
      YoDouble variable4 = new YoDouble("variable4", registry01);

      YoBoolean boolean1 = new YoBoolean("boolean1", root);
      YoBoolean boolean2 = new YoBoolean("boolean2", registry01);

      YoDouble repeatNameVariable_root = new YoDouble("repeatNameVariable", root);
      YoDouble repeatNameVariable_0 = new YoDouble("repeatNameVariable", registry0);
      YoDouble repeatNameVariable_00 = new YoDouble("repeatNameVariable", registry00);
      YoDouble repeatNameVariable_01 = new YoDouble("repeatNameVariable", registry01);

      YoDouble exceptionalVariable = new YoDouble("exceptionalVariable", root);

      variable1.set(0.1);
      variable2.set(0.2);
      variable3.set(0.3);
      variable4.set(0.4);

      boolean1.set(true);
      boolean2.set(true);

      repeatNameVariable_root.set(10.0);
      repeatNameVariable_0.set(20.0);
      repeatNameVariable_00.set(30.0);
      repeatNameVariable_01.set(40.0);

      exceptionalVariable.set(1234.0);

      return root;
   }
}
