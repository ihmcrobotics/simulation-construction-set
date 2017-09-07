package us.ihmc.simulationconstructionset.gui;

import java.awt.event.ItemEvent;
import java.awt.event.ItemListener;
import java.util.ArrayList;

import javax.swing.JCheckBoxMenuItem;

import us.ihmc.graphicsDescription.yoGraphics.YoGraphic;

public class YoGraphicCheckBoxMenuItem extends JCheckBoxMenuItem implements ItemListener
{
   private static final long serialVersionUID = -1641762511153430886L;
   private ArrayList<YoGraphic> yoGraphics = new ArrayList<YoGraphic>();

   public YoGraphicCheckBoxMenuItem(String label, ArrayList<YoGraphic> yoGraphics)
   {
      this(label, yoGraphics, true);
   }

   public YoGraphicCheckBoxMenuItem(String label, ArrayList<YoGraphic> yoGraphics, boolean selectedState)
   {
      super(label, selectedState);
      this.yoGraphics = yoGraphics;
      this.addItemListener(this);
   }

   public void addYoGraphics(ArrayList<YoGraphic> yoGraphics)
   {
      this.yoGraphics.addAll(yoGraphics);
   }

   @Override
   public void itemStateChanged(ItemEvent ie)
   {
      if (ie.getStateChange() == ItemEvent.SELECTED)
      {
         showYoGraphics();
      }
      else
      {
         hideYoGraphics();
      }
   }

   public void showYoGraphics()
   {
      this.setSelected(true);

      for (int i = 0; i < yoGraphics.size(); i++)
      {
         YoGraphic yoGraphic = yoGraphics.get(i);
         yoGraphic.showGraphicObject();
      }
   }

   public void hideYoGraphics()
   {
      this.setSelected(false);

      for (int i = 0; i < yoGraphics.size(); i++)
      {
         YoGraphic yoGraphic = yoGraphics.get(i);
         yoGraphic.hideGraphicObject();
      }
   }
}
