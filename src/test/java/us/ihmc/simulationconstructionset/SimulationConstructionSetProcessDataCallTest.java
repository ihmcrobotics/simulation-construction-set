package us.ihmc.simulationconstructionset;

import static us.ihmc.robotics.Assert.assertEquals;

import org.junit.jupiter.api.Test;

import us.ihmc.yoVariables.buffer.interfaces.YoBufferProcessor;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoDouble;

public class SimulationConstructionSetProcessDataCallTest
{
   private static final boolean DEBUG = false;
   private YoRegistry registry;

   @Test // timeout=1000
   public void testForwardCount()
   {
      Robot robot = new Robot("testRobot");
      SimulationConstructionSetParameters parameters = SimulationConstructionSetParameters.createFromSystemProperties();
      parameters.setCreateGUI(false);
      parameters.setDataBufferSize(8192);
      SimulationConstructionSet scs = new SimulationConstructionSet(robot, parameters);

      registry = new YoRegistry("testRegustry");
      YoDouble dataSet = new YoDouble("dataSet", registry);
      scs.addYoRegistry(registry);

      int startValue = 5;
      int maxValue = 15;
      //Generate and populate data
      for (int i = startValue; i < maxValue; i++)
      {
         dataSet.set(i);
         scs.tickAndUpdate();
      }

      //Crop data to get rid of the first data point
      scs.setOutPoint();
      scs.setCurrentIndex(1);
      scs.setInPoint();
      scs.cropBuffer();

      //Apply the Data Processing Class
      YoBufferProcessor counterProcessingFunction = new CounterProcessingFunction(registry, true);
      scs.applyDataProcessingFunction(counterProcessingFunction);

      //Exact data from Data Processing Class
      YoDouble counterVariable = ((CounterProcessingFunction) counterProcessingFunction).getCountVariable();

      if (DEBUG)
      {
         System.out.println("===testForwardCount===");
         for (int i = 0; i < maxValue - startValue; i++)
         {
            scs.setCurrentIndex(i);
            System.out.println(dataSet.getDoubleValue() + "\t" + counterVariable.getDoubleValue());
         }
      }

      //Test
      double testNum1, testNum2;
      for (int i = 0; i < maxValue - startValue; i++)
      {
         scs.setCurrentIndex(i);
         testNum1 = dataSet.getDoubleValue();
         testNum2 = counterVariable.getDoubleValue();

         assertEquals(testNum1 - testNum2, startValue, 0);
      }
      scs.closeAndDispose();
   }

   @Test // timeout=1000
   public void testBackwardCount()
   {
      Robot robot = new Robot("testRobot");
      SimulationConstructionSetParameters parameters = SimulationConstructionSetParameters.createFromSystemProperties();
      parameters.setCreateGUI(false);
      parameters.setDataBufferSize(8192);
      SimulationConstructionSet scs = new SimulationConstructionSet(robot, parameters);

      registry = new YoRegistry("testRegustry");
      YoDouble dataSet = new YoDouble("dataSet", registry);
      scs.addYoRegistry(registry);

      int startValue = 5;
      int maxValue = 15;
      //Generate and populate data
      for (int i = startValue; i < maxValue; i++)
      {
         dataSet.set(i);
         scs.tickAndUpdate();
      }

      //Crop data to get rid of the first data point
      scs.setOutPoint();
      scs.setCurrentIndex(1);
      scs.setInPoint();
      scs.cropBuffer();

      //Apply the Data Processing Class
      YoBufferProcessor counterProcessingFunction = new CounterProcessingFunction(registry, false);
      scs.applyDataProcessingFunction(counterProcessingFunction);

      //Exact data from Data Processing Class
      YoDouble counterVariable = ((CounterProcessingFunction) counterProcessingFunction).getCountVariable();

      if (DEBUG)
      {
         System.out.println("===testBackwardCount===");

         for (int i = 0; i < maxValue - startValue; i++)
         {
            scs.setCurrentIndex(i);
            System.out.println(dataSet.getDoubleValue() + "\t" + counterVariable.getDoubleValue());
         }
      }

      //Test
      double testNum1, testNum2;
      for (int i = 0; i < maxValue - startValue; i++)
      {
         scs.setCurrentIndex(i);
         testNum1 = dataSet.getDoubleValue();
         testNum2 = counterVariable.getDoubleValue();

         assertEquals(testNum1 + testNum2, maxValue - 1, 0);
      }
      scs.closeAndDispose();
   }

   @Test // timeout=1000
   public void testForwardCopy()
   {
      Robot robot = new Robot("testRobot");
      SimulationConstructionSetParameters parameters = SimulationConstructionSetParameters.createFromSystemProperties();
      parameters.setCreateGUI(false);
      parameters.setDataBufferSize(8192);
      SimulationConstructionSet scs = new SimulationConstructionSet(robot, parameters);

      registry = new YoRegistry("testRegustry");
      YoDouble dataSet = new YoDouble("dataSet", registry);
      scs.addYoRegistry(registry);

      int startValue = 5;
      int maxValue = 15;
      //Generate and populate data
      for (int i = startValue; i < maxValue; i++)
      {
         dataSet.set(i);
         scs.tickAndUpdate();
      }

      //Crop data to get rid of the first data point
      scs.setOutPoint();
      scs.setCurrentIndex(1);
      scs.setInPoint();
      scs.cropBuffer();

      //Apply the Data Processing Class
      YoBufferProcessor copierProcessingFunction = new CopierProcessingFunction(dataSet, registry, true);
      scs.applyDataProcessingFunction(copierProcessingFunction);

      //Exact data from Data Processing Class
      YoDouble copierVariable = ((CopierProcessingFunction) copierProcessingFunction).getCopyVariable();

      if (DEBUG)
      {
         System.out.println("===testForwardCopy===");
         for (int i = 0; i < maxValue - startValue; i++)
         {
            scs.setCurrentIndex(i);
            System.out.println(dataSet.getDoubleValue() + "\t" + copierVariable.getDoubleValue());
         }
      }

      //Test
      double testNum1, testNum2;
      for (int i = 0; i < maxValue - startValue; i++)
      {
         scs.setCurrentIndex(i);
         testNum1 = dataSet.getDoubleValue();
         testNum2 = copierVariable.getDoubleValue();

         assertEquals(testNum1 - testNum2, 0, 0);
      }
      scs.closeAndDispose();
   }

   @Test // timeout=1000
   public void testBackwardCopy()
   {
      Robot robot = new Robot("testRobot");
      SimulationConstructionSetParameters parameters = SimulationConstructionSetParameters.createFromSystemProperties();
      parameters.setCreateGUI(false);
      parameters.setDataBufferSize(8192);
      SimulationConstructionSet scs = new SimulationConstructionSet(robot, parameters);

      registry = new YoRegistry("testRegustry");
      YoDouble dataSet = new YoDouble("dataSet", registry);
      scs.addYoRegistry(registry);

      int startValue = 5;
      int maxValue = 15;
      //Generate and populate data
      for (int i = startValue; i < maxValue; i++)
      {
         dataSet.set(i);
         scs.tickAndUpdate();
      }

      //Crop data to get rid of the first data point
      scs.setOutPoint();
      scs.setCurrentIndex(1);
      scs.setInPoint();
      scs.cropBuffer();

      //Apply the Data Processing Class
      YoBufferProcessor copierProcessingFunction = new CopierProcessingFunction(dataSet, registry, false);
      scs.applyDataProcessingFunction(copierProcessingFunction);

      //Exact data from Data Processing Class
      YoDouble copierVariable = ((CopierProcessingFunction) copierProcessingFunction).getCopyVariable();

      if (DEBUG)
      {
         System.out.println("===testBackwardCopy===");

         for (int i = 0; i < maxValue - startValue; i++)
         {
            scs.setCurrentIndex(i);
            System.out.println(dataSet.getDoubleValue() + "\t" + copierVariable.getDoubleValue());
         }
      }

      //Test
      double testNum1, testNum2;
      for (int i = 0; i < maxValue - startValue; i++)
      {
         scs.setCurrentIndex(i);
         testNum1 = dataSet.getDoubleValue();
         testNum2 = copierVariable.getDoubleValue();

         assertEquals(testNum1 - testNum2, 0, 0);
      }
      scs.closeAndDispose();
   }

   public static class CopierProcessingFunction implements YoBufferProcessor
   {
      private final boolean forward;
      private final YoDouble copyVariable;
      private final YoDouble testVariable;

      public CopierProcessingFunction(YoDouble inputData, YoRegistry registry, boolean forward)
      {
         this.forward = forward;
         testVariable = inputData;
         copyVariable = new YoDouble("copyVariable", registry);
      }

      @Override
      public boolean goForward()
      {
         return forward;
      }

      @Override
      public void process(int startIndex, int endIndex, int currentIndex)
      {
         double holderDouble;

         holderDouble = testVariable.getDoubleValue();
         copyVariable.set(holderDouble);
      }

      public YoDouble getCopyVariable()
      {
         return copyVariable;
      }
   }

   public static class CounterProcessingFunction implements YoBufferProcessor
   {
      private final boolean forward;
      private final YoDouble countVariable;
      private int count = 0;

      public CounterProcessingFunction(YoRegistry registry, boolean forward)
      {
         this.forward = forward;
         countVariable = new YoDouble("countVariable", registry);
      }

      @Override
      public boolean goForward()
      {
         return forward;
      }

      @Override
      public void process(int startIndex, int endIndex, int currentIndex)
      {
         countVariable.set(count);
         count++;
      }

      public YoDouble getCountVariable()
      {
         return countVariable;
      }
   }
}
