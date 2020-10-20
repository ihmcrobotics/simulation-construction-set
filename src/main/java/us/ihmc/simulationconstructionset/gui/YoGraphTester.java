package us.ihmc.simulationconstructionset.gui;

import javax.swing.JFrame;

import us.ihmc.graphicsDescription.graphInterfaces.GraphIndicesHolder;
import us.ihmc.graphicsDescription.graphInterfaces.SelectedVariableHolder;
import us.ihmc.yoVariables.buffer.YoBufferVariableEntry;
import us.ihmc.yoVariables.buffer.interfaces.YoBufferVariableEntryHolder;
import us.ihmc.yoVariables.buffer.interfaces.YoTimeBufferHolder;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoDouble;

public class YoGraphTester
{
   public void testYoGraph()
   {
      SelectedVariableHolder selectedVariableHolder = new SelectedVariableHolder();

      JFrame jFrame = new JFrame("testYoGraph");
      YoGraphRemover yoGraphRemover = yoGraph -> { };
      YoBufferVariableEntryHolder dataEntryHolder = yoVariable -> null;
      YoTimeBufferHolder timeDataHolder = new MinimalTimeDataHolder(200);
      GraphIndicesHolder graphIndicesHolder = new MinimalGraphIndicesHolder();

      YoGraph yoGraph = new YoGraph(graphIndicesHolder, yoGraphRemover, selectedVariableHolder, dataEntryHolder, timeDataHolder, jFrame);

      int nPoints = 200;
      YoRegistry registry = new YoRegistry("registry");
      YoDouble yoVariable = new YoDouble("variableOne", registry);

      YoBufferVariableEntry dataEntry = new YoBufferVariableEntry(yoVariable, nPoints);

      double value = 0.0;

      for (int i = 0; i < nPoints; i++)
      {
         yoVariable.set(value);
         value = value + 0.001;
         dataEntry.writeIntoBufferAt(i);
      }

      yoGraph.addVariable(dataEntry);

      jFrame.getContentPane().add(yoGraph);
      jFrame.setSize(800, 200);
      jFrame.setVisible(true);
   }

   public static void main(String[] args)
   {
      new YoGraphTester().testYoGraph();
   }
}
