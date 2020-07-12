package us.ihmc.simulationconstructionset.gui;

import java.awt.event.ActionEvent;
import java.awt.event.ActionListener;
import java.awt.event.MouseEvent;
import java.awt.event.MouseListener;

import javax.swing.BoxLayout;
import javax.swing.JLabel;
import javax.swing.JPanel;
import javax.swing.JSlider;
import javax.swing.JTextField;
import javax.swing.SwingConstants;
import javax.swing.event.ChangeEvent;
import javax.swing.event.ChangeListener;

import us.ihmc.yoVariables.listener.YoVariableChangedListener;
import us.ihmc.yoVariables.registry.YoTools;
import us.ihmc.yoVariables.variable.YoVariable;

@SuppressWarnings("serial")
public class YoSliderpanel extends JPanel implements MouseListener, YoVariableChangedListener
{
   private static final int SHORT_NAME_LENGTH = 20;
   private YoVariable var;
   private double min = 0.0;
   private double max = 0.0;
   private JTextField maxField;
   private JTextField minField;
   private JLabel name;
   private double defaultMinMaxOffset = 10.0;
   private JLabel value;
   private JSlider slider;

   double precision = 1000;

   public YoSliderpanel(YoVariable var)
   {
      this.var = var;
      setLayout(new BoxLayout(this, BoxLayout.Y_AXIS));
      setup();
      var.addListener(this);
   }

   private int convertDoubleToPrecisionInt(double value)
   {
      double precisionValue = value * precision;

      String newVal = precisionValue + "";

      int decimalLocal = newVal.indexOf(".");

      if (decimalLocal != -1)

      {
         newVal = newVal.substring(0, decimalLocal);
      }

      System.out.println("newVal" + newVal);

      return new Integer(newVal);
   }

   private void setup()
   {
      name = new JLabel(YoTools.shortenString(var.getName(), SHORT_NAME_LENGTH));
      min = var.getValueAsDouble() - defaultMinMaxOffset;
      max = var.getValueAsDouble() + defaultMinMaxOffset;

      slider = new JSlider(SwingConstants.VERTICAL,
                           convertDoubleToPrecisionInt(min),
                           convertDoubleToPrecisionInt(max),
                           convertDoubleToPrecisionInt(var.getValueAsDouble()));
      setUpMax();
      setUpMin();

      value = new JLabel(var.getValueAsDouble() + "");

      slider.addChangeListener(new ChangeListener()
      {
         @Override
         public void stateChanged(ChangeEvent e)
         {
            double actualSliderValue = (new Double(slider.getValue()) / precision);
            value.setText(actualSliderValue + "");
            var.setValueFromDouble(actualSliderValue);
         }
      });
      this.add(name);
      this.add(maxField);
      this.add(slider);
      this.add(minField);
      this.add(value);
   }

   private void setUpMax()
   {
      maxField = new JTextField(max + "");
      maxField.addActionListener(new ActionListener()
      {
         @Override
         public void actionPerformed(ActionEvent e)
         {
            double newValue = new Double(maxField.getText());
            if (newValue >= var.getValueAsDouble())
            {
               max = newValue;
               slider.setMaximum(convertDoubleToPrecisionInt(max));
            }
            else
            {
               System.err.println("cant set max to a value less then the current value");
               maxField.setText(max + "");
            }

         }
      });
   }

   private void setUpMin()
   {
      minField = new JTextField(min + "");
      minField.addActionListener(new ActionListener()
      {
         @Override
         public void actionPerformed(ActionEvent e)
         {
            double newValue = new Double(minField.getText());
            if (newValue <= var.getValueAsDouble())
            {
               min = newValue;
               slider.setMinimum(convertDoubleToPrecisionInt(min));
            }
            else
            {
               System.err.println("cant set min to a value greater then the current value");
               minField.setText(min + "");
            }

         }
      });
   }

   @Override
   public void mouseClicked(MouseEvent e)
   {

   }

   @Override
   public void mousePressed(MouseEvent e)
   {

   }

   @Override
   public void mouseReleased(MouseEvent e)
   {

   }

   @Override
   public void mouseEntered(MouseEvent e)
   {

   }

   @Override
   public void mouseExited(MouseEvent e)
   {

   }

   private void setSliderValueOnVariableChange()
   {
      if (var.getValueAsDouble() < min)
      {
         min = var.getValueAsDouble();
         slider.setMinimum(convertDoubleToPrecisionInt(min));
         minField.setText(min + "");

      }

      if (var.getValueAsDouble() > max)
      {
         max = var.getValueAsDouble();
         slider.setMaximum(convertDoubleToPrecisionInt(max));
         maxField.setText(max + "");
      }

      slider.setValue(convertDoubleToPrecisionInt(var.getValueAsDouble()));
   }

   @Override
   public void changed(YoVariable v)
   {
      value.setText(v.getValueAsDouble() + "");
      setSliderValueOnVariableChange();

   }

}
