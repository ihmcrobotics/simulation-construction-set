package us.ihmc.simulationconstructionset.gui.hierarchyTree;

import java.awt.BorderLayout;
import java.awt.GridLayout;
import java.awt.event.ActionEvent;
import java.awt.event.ActionListener;

import javax.swing.JPanel;
import javax.swing.JTextField;
import javax.swing.event.DocumentEvent;
import javax.swing.event.DocumentListener;

public class NamespaceSearchPanel extends JPanel
{
   private static final long serialVersionUID = 2531114756584430672L;
   private NamespaceHierarchyTree namespaceHierarchyTree;

   public NamespaceSearchPanel(NamespaceHierarchyTree namespaceHierarchyTree)
   {
      super(new BorderLayout());

      setName("NamespaceSearchPanel");
      this.add(new NamespaceSearchField(), BorderLayout.NORTH);
      this.namespaceHierarchyTree = namespaceHierarchyTree;
      this.add(namespaceHierarchyTree);
   }

   public class NamespaceSearchField extends JPanel implements ActionListener
   {
      private static final long serialVersionUID = 8594680043494017825L;
      private final JTextField searchTextField;

      public NamespaceSearchField()
      {
         setLayout(new GridLayout(1, 1));
         searchTextField = new JTextField();
         searchTextField.addActionListener(this);

         DocumentListener documentListener = new DocumentListener()
         {
            @Override
            public void insertUpdate(DocumentEvent e)
            {
               filterNamespaceTree();
            }

            @Override
            public void removeUpdate(DocumentEvent e)
            {
               filterNamespaceTree();
            }

            @Override
            public void changedUpdate(DocumentEvent e)
            {
            }
         };

         searchTextField.getDocument().addDocumentListener(documentListener);

         this.add(searchTextField);
      }

      @Override
      public void actionPerformed(ActionEvent arg0)
      {
         filterNamespaceTree();
      }

      private void filterNamespaceTree()
      {
         namespaceHierarchyTree.filter(searchTextField.getText().toString());
      }
   }
}
