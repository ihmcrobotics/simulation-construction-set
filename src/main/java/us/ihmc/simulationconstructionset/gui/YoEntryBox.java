package us.ihmc.simulationconstructionset.gui;

import java.awt.Color;
import java.awt.Dimension;
import java.awt.dnd.DropTarget;
import java.awt.event.ActionEvent;
import java.awt.event.ActionListener;
import java.awt.event.FocusEvent;
import java.awt.event.FocusListener;
import java.awt.event.MouseEvent;
import java.awt.event.MouseListener;
import java.util.ArrayList;

import javax.swing.BoxLayout;
import javax.swing.JMenuItem;
import javax.swing.JPanel;
import javax.swing.JPopupMenu;
import javax.swing.SwingUtilities;
import javax.swing.TransferHandler;
import javax.swing.event.ChangeEvent;
import javax.swing.event.ChangeListener;

import us.ihmc.graphicsDescription.graphInterfaces.SelectedVariableHolder;
import us.ihmc.simulationconstructionset.SimulationConstructionSet;
import us.ihmc.yoVariables.listener.YoVariableChangedListener;
import us.ihmc.yoVariables.variable.YoEnum;
import us.ihmc.yoVariables.variable.YoVariable;

public class YoEntryBox extends JPanel implements MouseListener, ActionListener, FocusListener, ChangeListener
{
   private final boolean USE_NEW_DISPLAY_FOR_ENUMS = true;

   protected static final String DEFAULT_UNBOUND_ENTRY_BOX_LABEL = "UNUSED";

   private static final long serialVersionUID = -3598041913472018651L;

   protected static final int COMPONENT_HEIGHT = 26;

   protected static final int MAX_COMPONENT_LENGTH = 400;

   protected static final int MIN_COMPONENT_LENGTH = 50;

   private Color origionalColor = null;

   private static ArrayList<YoVariableChangedListener> variableChangedListeners = new ArrayList<>();

   private YoVariableEntryContainer activeEntryContainer = new YoTextEntryContainer();

   private EntryBoxArrayPanel entryBoxArrayPanel;

   private JPopupMenu popupMenu;

   private SelectedVariableHolder selectedVariableHolder;

   public YoEntryBox(EntryBoxArrayPanel entryBoxArrayPanel, SelectedVariableHolder holder)
   {
      super();

      setLayout(new BoxLayout(this, BoxLayout.X_AXIS));
      selectedVariableHolder = holder;
      this.entryBoxArrayPanel = entryBoxArrayPanel;
      setOpaque(true);
      setName(DEFAULT_UNBOUND_ENTRY_BOX_LABEL);
      activeEntryContainer.setup(this);

      addFocusListener(this);

      addMouseListener(this);
      if (!SimulationConstructionSet.DISABLE_DnD)
      {
         setDropTarget(new DropTarget(this, new YoEntryBoxTargetListener(this)));
         setTransferHandler(new YoEntryBoxTransferHandler());
      }

      popupMenu = new ForcedRepaintPopupMenu();
      JMenuItem delete = new JMenuItem("Delete Entry Box");
      delete.addActionListener(new ActionListener()
      {
         @Override
         public void actionPerformed(ActionEvent e)
         {
            System.out.println(e);
            YoVariable variable = activeEntryContainer.getVariable();
            if (variable != null)
               removeVariable(variable);
         }
      });
      popupMenu.add(delete);
      //      this.setToolTipText(DEFAULT_UNBOUND_ENTRY_BOX_LABEL);
      setToolTipText("");
      setPreferredSize(new Dimension(200, 26));
      origionalColor = getBackground();
   }

   public static void attachVariableChangedListener(YoVariableChangedListener listener)
   {
      variableChangedListeners.add(listener);
   }

   protected static void informVariableChangedListeners(YoVariable variableChanged)
   {
      for (int i = 0; i < variableChangedListeners.size(); i++)
      {
         YoVariableChangedListener listener = variableChangedListeners.get(i);
         listener.changed(variableChanged);
      }
   }

   public static void removeVariableChangedListener(YoVariableChangedListener listener)
   {
      variableChangedListeners.remove(listener);
   }

   @Override
   public void actionPerformed(ActionEvent evt)
   {
      //    System.out.println("YoEntryBox: Hey, I haz an actionPerformed!");
      activeEntryContainer.actionPerformed(this, evt);
   }

   public void addVariable(YoVariable variable)
   {
      if ((entryBoxArrayPanel != null) && entryBoxArrayPanel.isHoldingVariable(variable))
         return;
      setVariableInThisBox(variable);

      updateActiveContainer();

      if (entryBoxArrayPanel != null)
         entryBoxArrayPanel.checkStatus();
   }

   @Override
   public void focusGained(FocusEvent evt)
   {
      if (activeEntryContainer.getVariable() != null)
      {
         activeEntryContainer.focusGained(this);
      }
      else
         passOnFocusRequest();
   }

   @Override
   public void focusLost(FocusEvent evt)
   {
      if (activeEntryContainer.isEventSource(this, evt))
      {
         if (activeEntryContainer.getVariable() != null)
         {
            activeEntryContainer.focusLost(this);
         }
      }
      else
      {
         if (evt.getSource().equals(this))
         {
            if (popupMenu.isVisible())
            {
               popupMenu.setVisible(false);
            }
         }
      }
   }

   public int getNumVars()
   {
      if (activeEntryContainer.getVariable() != null)
         return 1;
      else
         return 0;
   }

   public SelectedVariableHolder getSelectedVariableHolder()
   {
      return selectedVariableHolder;
   }

   public YoVariable getVariableInThisBox()
   {
      return activeEntryContainer.getVariable();
   }

   public boolean isHoldingVariable(YoVariable v)
   {
      return (activeEntryContainer.getVariable() == v);
   }

   @Override
   public void mouseClicked(MouseEvent evt)
   {
   }

   @Override
   public void mouseEntered(MouseEvent evt)
   {
   }

   @Override
   public void mouseExited(MouseEvent evt)
   {
   }

   @Override
   public void mousePressed(MouseEvent evt)
   {
      this.requestFocus();

      if (SwingUtilities.isRightMouseButton(evt))
      {
         popupMenu.setLocation(evt.getXOnScreen(), evt.getYOnScreen());
         popupMenu.setVisible(true);
      }

      // Left click places index:

      else
      {
         if (activeEntryContainer.getVariable() == null)
         {
            // YoVariable v = VarPanel.selectedVariable;

            YoVariable v = selectedVariableHolder.getSelectedVariable();
            if (v != null)
               addVariable(v);
         }
         else
         {
            selectedVariableHolder.setSelectedVariable(activeEntryContainer.getVariable());

            if (!SimulationConstructionSet.DISABLE_DnD)
            {
               if (!evt.isControlDown())
               {
                  getTransferHandler().exportAsDrag(this, evt, TransferHandler.MOVE);
                  YoGraph.setActionPerformedByDragAndDrop(TransferHandler.MOVE);
               }
               else if (evt.isControlDown())
               {
                  getTransferHandler().exportAsDrag(this, evt, TransferHandler.COPY);
                  YoGraph.setActionPerformedByDragAndDrop(TransferHandler.COPY);
               }
               
               YoGraph.setSourceOfDrag(this);
            }
         }
      }

   }

   @Override
   public void mouseReleased(MouseEvent evt)
   {
   }

   protected void passOnFocusRequest()
   {
      if (entryBoxArrayPanel != null)
         entryBoxArrayPanel.requestFocus();
      else
         getParent().requestFocus();
   }

   public void removeVariable(YoVariable variable)
   {
      activeEntryContainer.removeVariable(variable);

      setToolTipText("");
      setName("");
      //      this.setToolTipText(DEFAULT_UNBOUND_ENTRY_BOX_LABEL);

      if (entryBoxArrayPanel != null)
         entryBoxArrayPanel.checkStatus();

      if (popupMenu.isVisible())
         popupMenu.setVisible(false);

      updateUI();
   }

   @SuppressWarnings({"rawtypes"})
   private void setContainerType(YoVariable variable)
   {
      if ((variable instanceof YoEnum<?>) && USE_NEW_DISPLAY_FOR_ENUMS)
      {
         YoEnum<?> enumVariable = (YoEnum<?>) variable;
         switchContainerType(new YoEnumEntryContainer(enumVariable.getEnumValuesAsString()));
      }
      else
      {
         switchContainerType(new YoTextEntryContainer());
      }
   }

   private void switchContainerType(YoVariableEntryContainer yoEntryContainer)
   {
      //    if (activeEntryContainer.getClass() != yoEntryContainer.getClass())
      //    {
      activeEntryContainer.shutdown(this);
      activeEntryContainer = yoEntryContainer;
      activeEntryContainer.setup(this);

      //    }
   }

   public synchronized void setTextField()
   {
      updateActiveContainer();
   }

   public void setVariableInThisBox(YoVariable variableInThisBox)
   {
      setContainerType(variableInThisBox);
      activeEntryContainer.bindToVariable(this, variableInThisBox);
      updateToolTipText(variableInThisBox);
      setName(variableInThisBox.getName() + "_YoEntryBox");
   }

   public void updateActiveContainer()
   {
      final YoEntryBox thisEntryBox = this;

      EventDispatchThreadHelper.invokeLater(new Runnable()
      {
         @Override
         public void run()
         {
            activeEntryContainer.update(thisEntryBox);
         }
      });
   }

   protected void updateToolTipText(YoVariable variableInThisBox)
   {
      String toolTip = variableInThisBox.getDescription();
      if ((toolTip == null) || toolTip.equals(""))
         toolTip = variableInThisBox.getFullNameString();
      setToolTipText(toolTip);
   }

   public YoVariableEntryContainer getActiveYoVariableEntryContainer()
   {
      return activeEntryContainer;
   }

   @Override
   public void stateChanged(ChangeEvent e)
   {
      YoVariable tmp = ((SelectedVariableHolder) e.getSource()).getSelectedVariable();
      if (activeEntryContainer.getVariable() != null)
      {
         if (activeEntryContainer.getVariable().equals(tmp))
            setBackground(Color.GREEN);
         else
            setBackground(origionalColor);
      }
   }

}
