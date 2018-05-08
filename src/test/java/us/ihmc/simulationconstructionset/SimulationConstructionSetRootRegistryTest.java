package us.ihmc.simulationconstructionset;

import static org.junit.Assert.*;

import org.junit.Test;

import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationPlan;
import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationTest;
import us.ihmc.continuousIntegration.IntegrationCategory;
import us.ihmc.yoVariables.dataBuffer.DataBuffer;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.simulationconstructionset.Robot;
import us.ihmc.simulationconstructionset.SimulationConstructionSet;
import us.ihmc.simulationconstructionset.SimulationConstructionSetParameters;
import us.ihmc.simulationconstructionset.gui.StandardSimulationGUI;
import us.ihmc.simulationconstructionset.gui.YoVariableExplorerTabbedPane;
import us.ihmc.simulationconstructionset.gui.yoVariableSearch.YoVariablePanel;

@ContinuousIntegrationPlan(categories={IntegrationCategory.UI})
public class SimulationConstructionSetRootRegistryTest
{
   private static final boolean SHOW_GUI = false;

	@ContinuousIntegrationTest(estimatedDuration = 1.1)
	@Test(timeout = 30000)
   public void testRootRegistryNothingFancy()
   {
      Robot robot = new Robot("RobotsRootRegistry");

      YoVariableRegistry registryOne = new YoVariableRegistry("RegistryOne");
      robot.getRobotsYoVariableRegistry().addChild(registryOne);
      YoDouble variableOne = new YoDouble("variableOne", registryOne);
      
      SimulationConstructionSetParameters parameters = new SimulationConstructionSetParameters();
      parameters.setCreateGUI(SHOW_GUI);
      SimulationConstructionSet scs = new SimulationConstructionSet(robot, parameters);
      sleep(1000);
      scs.startOnAThread();
      
      YoVariableRegistry rootRegistry = scs.getRootRegistry();
      DataBuffer dataBuffer = scs.getDataBuffer();

      assertTrue(variableOne == rootRegistry.getVariable("variableOne"));
      assertTrue(variableOne == dataBuffer.getVariable("variableOne"));

      if (SHOW_GUI)
      {
         StandardSimulationGUI standardSimulationGUI = scs.getStandardSimulationGUI();
         YoVariableExplorerTabbedPane combinedVarPanel = standardSimulationGUI.getCombinedVarPanel();

         // This also fails when the Search Panel doesn't come up...
         sleep(2000);  //+++JEP: Not sure why need this sleep, but it fails if we don't...

         combinedVarPanel.setVisibleVarPanel("root.RobotsRootRegistry.RegistryOne");
         YoVariablePanel visibleVarPanel = combinedVarPanel.getVisibleVarPanel();
        
         assertTrue(visibleVarPanel != null);
         assertTrue(variableOne == visibleVarPanel.getYoVariable("variableOne"));
      }
      
      scs.closeAndDispose();
   }

	@ContinuousIntegrationTest(estimatedDuration = 0.1)
	@Test(timeout = 30000) 
   public void testVarGroups()
   {
      Robot robot = new Robot("testVarGroups");
      
      YoVariableRegistry registryOne = new YoVariableRegistry("registryOne");
      YoVariableRegistry registryTwo = new YoVariableRegistry("registryTwo");

      YoDouble variableOneA = new YoDouble("variableOneA", registryOne);
      YoDouble variableOneB = new YoDouble("variableOneB", registryOne);
      
      YoDouble variableTwoA = new YoDouble("variableTwoA", registryTwo);
      YoDouble variableTwoB = new YoDouble("variableTwoB", registryTwo);
      
      robot.getRobotsYoVariableRegistry().addChild(registryOne);
      robot.getRobotsYoVariableRegistry().addChild(registryTwo);
      
      SimulationConstructionSetParameters parameters = new SimulationConstructionSetParameters();
      parameters.setCreateGUI(SHOW_GUI);
      SimulationConstructionSet scs = new SimulationConstructionSet(robot, parameters);
      scs.setupVarGroup("VarGroupToTest", new String[]{"variableOneA", "variableTwoB"});
      
      scs.startOnAThread();

      if (SHOW_GUI)
      {
         StandardSimulationGUI standardSimulationGUI = scs.getStandardSimulationGUI();
         YoVariableExplorerTabbedPane combinedVarPanel = standardSimulationGUI.getCombinedVarPanel();

         sleep(2000);  //+++JEP: Not sure why need this sleep, but it fails if we don't...
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

	@ContinuousIntegrationTest(estimatedDuration = 0.1)
	@Test(timeout = 30000)
   public void testRootRegistryAddYoVariablesAfterConstruction()
   {
      Robot robot = new Robot("TestAfterConstruction");
      
      YoVariableRegistry registryBeforeConstructionOne = new YoVariableRegistry("RegistryBeforeConstructionOne");
      robot.getRobotsYoVariableRegistry().addChild(registryBeforeConstructionOne);
      YoDouble variableBeforeConstructionOne = new YoDouble("variableBeforeConstructionOne", registryBeforeConstructionOne);
      
      YoVariableRegistry registryBeforeConstructionOneOne = new YoVariableRegistry("RegistryBeforeConstructionOneOne");
      registryBeforeConstructionOne.addChild(registryBeforeConstructionOneOne);
      YoDouble variableBeforeConstructionOneOne = new YoDouble("variableBeforeConstructionOneOne", registryBeforeConstructionOneOne);
      
      SimulationConstructionSetParameters parameters = new SimulationConstructionSetParameters();
      parameters.setCreateGUI(SHOW_GUI);
      SimulationConstructionSet scs = new SimulationConstructionSet(robot, parameters);
      
      YoDouble variableAfterConstructionZero = new YoDouble("variableAfterConstructionZero", registryBeforeConstructionOne);

      YoVariableRegistry registryAfterConstructionOne = new YoVariableRegistry("RegistryAfterConstructionOne");
      YoDouble variableAfterConstructionOne = new YoDouble("variableAfterConstructionOne", registryAfterConstructionOne);
      scs.getRootRegistry().addChild(registryAfterConstructionOne);
      YoDouble variableAfterConstructionTwo = new YoDouble("variableAfterConstructionTwo", registryAfterConstructionOne);
      
      scs.startOnAThread();
      
      YoDouble variableAfterThreadZero = new YoDouble("variableAfterThreadZero", registryAfterConstructionOne);
//      sleep(100000);

      YoVariableRegistry registryAfterThreadOne = new YoVariableRegistry("RegistryAfterThreadOne");
      YoDouble variableAfterThreadOne = new YoDouble("variableAfterThreadOne", registryAfterThreadOne);
      registryAfterConstructionOne.addChild(registryAfterThreadOne);
      YoDouble variableAfterThreadTwo = new YoDouble("variableAfterThreadTwo", registryAfterThreadOne);

      YoVariableRegistry rootRegistry = scs.getRootRegistry();
      
      // Make sure the variables are in the registry chain...
      assertTrue(variableBeforeConstructionOne == rootRegistry.getVariable("variableBeforeConstructionOne"));
      assertTrue(variableAfterConstructionZero == rootRegistry.getVariable("variableAfterConstructionZero"));
      assertTrue(variableAfterConstructionOne == rootRegistry.getVariable("variableAfterConstructionOne"));
      assertTrue(variableAfterConstructionTwo == rootRegistry.getVariable("variableAfterConstructionTwo"));
      assertTrue(variableAfterThreadZero == rootRegistry.getVariable("variableAfterThreadZero"));
      assertTrue(variableAfterThreadOne == rootRegistry.getVariable("variableAfterThreadOne"));
      assertTrue(variableAfterThreadTwo == rootRegistry.getVariable("variableAfterThreadTwo"));
      
      // Make sure the variables are in the DataBuffer:
      DataBuffer dataBuffer = scs.getDataBuffer();
      assertTrue(variableBeforeConstructionOne == dataBuffer.getVariable("variableBeforeConstructionOne"));
      assertTrue(variableAfterConstructionZero == dataBuffer.getVariable("variableAfterConstructionZero"));
      assertTrue(variableAfterConstructionOne == dataBuffer.getVariable("variableAfterConstructionOne"));
      assertTrue(variableAfterConstructionTwo == dataBuffer.getVariable("variableAfterConstructionTwo"));
      assertTrue(variableAfterThreadZero == dataBuffer.getVariable("variableAfterThreadZero"));
      assertTrue(variableAfterThreadOne == dataBuffer.getVariable("variableAfterThreadOne"));
      assertTrue(variableAfterThreadTwo == dataBuffer.getVariable("variableAfterThreadTwo"));

      // Make sure the variables are on the GUI:
      if (SHOW_GUI)
      {
         StandardSimulationGUI standardSimulationGUI = scs.getStandardSimulationGUI();
  
         sleep(2000);  //+++JEP: Not sure why need this sleep, but it fails if we don't...

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
      while(true)
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
