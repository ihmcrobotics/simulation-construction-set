package us.ihmc.simulationconstructionset;

import static us.ihmc.robotics.Assert.assertTrue;

import org.junit.jupiter.api.Test;

import us.ihmc.simulationconstructionset.gui.StandardSimulationGUI;
import us.ihmc.simulationconstructionset.gui.YoVariableExplorerTabbedPane;
import us.ihmc.simulationconstructionset.gui.yoVariableSearch.YoVariablePanel;
import us.ihmc.yoVariables.dataBuffer.DataBuffer;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoDouble;

public class SimulationConstructionSetRootRegistryTest
{
   private static final boolean SHOW_GUI = false;

   @Test // timeout = 30000
   public void testRootRegistryNothingFancy()
   {
      Robot robot = new Robot("RobotsRootRegistry");

      YoRegistry registryOne = new YoRegistry("RegistryOne");
      robot.getRobotsYoRegistry().addChild(registryOne);
      YoDouble variableOne = new YoDouble("variableOne", registryOne);

      SimulationConstructionSetParameters parameters = SimulationConstructionSetParameters.createFromSystemProperties();
      parameters.setCreateGUI(SHOW_GUI);
      SimulationConstructionSet scs = new SimulationConstructionSet(robot, parameters);
      sleep(1000);
      scs.startOnAThread();

      YoRegistry rootRegistry = scs.getRootRegistry();
      DataBuffer dataBuffer = scs.getDataBuffer();

      assertTrue(variableOne == rootRegistry.findVariable("variableOne"));
      assertTrue(variableOne == dataBuffer.findVariable("variableOne"));

      if (SHOW_GUI)
      {
         StandardSimulationGUI standardSimulationGUI = scs.getStandardSimulationGUI();
         YoVariableExplorerTabbedPane combinedVarPanel = standardSimulationGUI.getCombinedVarPanel();

         // This also fails when the Search Panel doesn't come up...
         sleep(2000); //+++JEP: Not sure why need this sleep, but it fails if we don't...

         combinedVarPanel.setVisibleVarPanel("root.RobotsRootRegistry.RegistryOne");
         YoVariablePanel visibleVarPanel = combinedVarPanel.getVisibleVarPanel();

         assertTrue(visibleVarPanel != null);
         assertTrue(variableOne == visibleVarPanel.getYoVariable("variableOne"));
      }

      scs.closeAndDispose();
   }

   @Test // timeout = 30000
   public void testVarGroups()
   {
      Robot robot = new Robot("testVarGroups");

      YoRegistry registryOne = new YoRegistry("registryOne");
      YoRegistry registryTwo = new YoRegistry("registryTwo");

      YoDouble variableOneA = new YoDouble("variableOneA", registryOne);
      YoDouble variableOneB = new YoDouble("variableOneB", registryOne);

      YoDouble variableTwoA = new YoDouble("variableTwoA", registryTwo);
      YoDouble variableTwoB = new YoDouble("variableTwoB", registryTwo);

      robot.getRobotsYoRegistry().addChild(registryOne);
      robot.getRobotsYoRegistry().addChild(registryTwo);

      SimulationConstructionSetParameters parameters = SimulationConstructionSetParameters.createFromSystemProperties();
      parameters.setCreateGUI(SHOW_GUI);
      SimulationConstructionSet scs = new SimulationConstructionSet(robot, parameters);
      scs.setupVarGroup("VarGroupToTest", new String[] {"variableOneA", "variableTwoB"});

      scs.startOnAThread();

      if (SHOW_GUI)
      {
         StandardSimulationGUI standardSimulationGUI = scs.getStandardSimulationGUI();
         YoVariableExplorerTabbedPane combinedVarPanel = standardSimulationGUI.getCombinedVarPanel();

         sleep(2000); //+++JEP: Not sure why need this sleep, but it fails if we don't...
         // This also fails when the Search Panel doesn't come up...

         standardSimulationGUI.selectVarGroup("VarGroupToTest");
         YoVariablePanel visibleVarPanel = combinedVarPanel.getVisibleVarPanel();
         assertTrue(visibleVarPanel.getName().equals("VarGroupToTest"));
         assertTrue(variableOneA == visibleVarPanel.getYoVariable("variableOneA"));
         assertTrue(variableTwoB == visibleVarPanel.getYoVariable("variableTwoB"));
         assertTrue(null == visibleVarPanel.getYoVariable("variableOneB"));
         assertTrue(null == visibleVarPanel.getYoVariable("variableTwoA"));

         //         sleepForever();
      }

      scs.closeAndDispose();
   }

   @Test // timeout = 30000
   public void testRootRegistryAddYoVariablesAfterConstruction()
   {
      Robot robot = new Robot("TestAfterConstruction");

      YoRegistry registryBeforeConstructionOne = new YoRegistry("RegistryBeforeConstructionOne");
      robot.getRobotsYoRegistry().addChild(registryBeforeConstructionOne);
      YoDouble variableBeforeConstructionOne = new YoDouble("variableBeforeConstructionOne", registryBeforeConstructionOne);

      YoRegistry registryBeforeConstructionOneOne = new YoRegistry("RegistryBeforeConstructionOneOne");
      registryBeforeConstructionOne.addChild(registryBeforeConstructionOneOne);
      YoDouble variableBeforeConstructionOneOne = new YoDouble("variableBeforeConstructionOneOne", registryBeforeConstructionOneOne);

      SimulationConstructionSetParameters parameters = SimulationConstructionSetParameters.createFromSystemProperties();
      parameters.setCreateGUI(SHOW_GUI);
      SimulationConstructionSet scs = new SimulationConstructionSet(robot, parameters);

      YoDouble variableAfterConstructionZero = new YoDouble("variableAfterConstructionZero", registryBeforeConstructionOne);

      YoRegistry registryAfterConstructionOne = new YoRegistry("RegistryAfterConstructionOne");
      YoDouble variableAfterConstructionOne = new YoDouble("variableAfterConstructionOne", registryAfterConstructionOne);
      scs.getRootRegistry().addChild(registryAfterConstructionOne);
      YoDouble variableAfterConstructionTwo = new YoDouble("variableAfterConstructionTwo", registryAfterConstructionOne);

      scs.startOnAThread();

      YoDouble variableAfterThreadZero = new YoDouble("variableAfterThreadZero", registryAfterConstructionOne);
      //      sleep(100000);

      YoRegistry registryAfterThreadOne = new YoRegistry("RegistryAfterThreadOne");
      YoDouble variableAfterThreadOne = new YoDouble("variableAfterThreadOne", registryAfterThreadOne);
      registryAfterConstructionOne.addChild(registryAfterThreadOne);
      YoDouble variableAfterThreadTwo = new YoDouble("variableAfterThreadTwo", registryAfterThreadOne);

      YoRegistry rootRegistry = scs.getRootRegistry();

      // Make sure the variables are in the registry chain...
      assertTrue(variableBeforeConstructionOne == rootRegistry.findVariable("variableBeforeConstructionOne"));
      assertTrue(variableAfterConstructionZero == rootRegistry.findVariable("variableAfterConstructionZero"));
      assertTrue(variableAfterConstructionOne == rootRegistry.findVariable("variableAfterConstructionOne"));
      assertTrue(variableAfterConstructionTwo == rootRegistry.findVariable("variableAfterConstructionTwo"));
      assertTrue(variableAfterThreadZero == rootRegistry.findVariable("variableAfterThreadZero"));
      assertTrue(variableAfterThreadOne == rootRegistry.findVariable("variableAfterThreadOne"));
      assertTrue(variableAfterThreadTwo == rootRegistry.findVariable("variableAfterThreadTwo"));

      // Make sure the variables are in the DataBuffer:
      DataBuffer dataBuffer = scs.getDataBuffer();
      assertTrue(variableBeforeConstructionOne == dataBuffer.findVariable("variableBeforeConstructionOne"));
      assertTrue(variableAfterConstructionZero == dataBuffer.findVariable("variableAfterConstructionZero"));
      assertTrue(variableAfterConstructionOne == dataBuffer.findVariable("variableAfterConstructionOne"));
      assertTrue(variableAfterConstructionTwo == dataBuffer.findVariable("variableAfterConstructionTwo"));
      assertTrue(variableAfterThreadZero == dataBuffer.findVariable("variableAfterThreadZero"));
      assertTrue(variableAfterThreadOne == dataBuffer.findVariable("variableAfterThreadOne"));
      assertTrue(variableAfterThreadTwo == dataBuffer.findVariable("variableAfterThreadTwo"));

      // Make sure the variables are on the GUI:
      if (SHOW_GUI)
      {
         StandardSimulationGUI standardSimulationGUI = scs.getStandardSimulationGUI();

         sleep(2000); //+++JEP: Not sure why need this sleep, but it fails if we don't...

         YoVariableExplorerTabbedPane combinedVarPanel = standardSimulationGUI.getCombinedVarPanel();
         combinedVarPanel.setVisibleVarPanel("root.TestAfterConstruction.RegistryBeforeConstructionOne");
         //         sleep(2000);

         YoVariablePanel visibleVarPanel = combinedVarPanel.getVisibleVarPanel();
         System.out.println("visibleVarPanel = " + visibleVarPanel.getName());
         assertTrue(visibleVarPanel.getName().equals("RegistryBeforeConstructionOne"));
         assertTrue(variableBeforeConstructionOne == visibleVarPanel.getYoVariable("variableBeforeConstructionOne"));
         assertTrue(variableAfterConstructionZero == visibleVarPanel.getYoVariable("variableAfterConstructionZero"));

         combinedVarPanel.setVisibleVarPanel("root.TestAfterConstruction.RegistryBeforeConstructionOne.RegistryBeforeConstructionOneOne");
         visibleVarPanel = combinedVarPanel.getVisibleVarPanel();
         assertTrue(visibleVarPanel.getName().equals("RegistryBeforeConstructionOneOne"));
         assertTrue(variableBeforeConstructionOneOne == visibleVarPanel.getYoVariable("variableBeforeConstructionOneOne"));

         combinedVarPanel.setVisibleVarPanel("root.RegistryAfterConstructionOne");
         visibleVarPanel = combinedVarPanel.getVisibleVarPanel();
         assertTrue(visibleVarPanel.getName().equals("RegistryAfterConstructionOne"));

         assertTrue(variableAfterConstructionOne == visibleVarPanel.getYoVariable("variableAfterConstructionOne"));
         assertTrue(variableAfterConstructionTwo == visibleVarPanel.getYoVariable("variableAfterConstructionTwo"));
         assertTrue(variableAfterThreadZero == visibleVarPanel.getYoVariable("variableAfterThreadZero"));

         combinedVarPanel.setVisibleVarPanel("root.RegistryAfterConstructionOne.RegistryAfterThreadOne");
         visibleVarPanel = combinedVarPanel.getVisibleVarPanel();
         assertTrue(visibleVarPanel.getName().equals("RegistryAfterThreadOne"));

         assertTrue(variableAfterThreadOne == visibleVarPanel.getYoVariable("variableAfterThreadOne"));
         assertTrue(variableAfterThreadTwo == visibleVarPanel.getYoVariable("variableAfterThreadTwo"));

         sleepForever();
      }

      scs.closeAndDispose();
   }

   private static void sleepForever()
   {
      while (true)
      {
         sleep(10000);
      }
   }

   private static void sleep(long sleepMillis)
   {
      try
      {
         Thread.sleep(sleepMillis);
      }
      catch (InterruptedException e)
      {
      }

   }
}
