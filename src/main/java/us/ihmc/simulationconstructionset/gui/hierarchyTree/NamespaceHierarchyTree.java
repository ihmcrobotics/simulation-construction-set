package us.ihmc.simulationconstructionset.gui.hierarchyTree;

import java.awt.Component;
import java.awt.Graphics;
import java.awt.event.ActionEvent;
import java.awt.event.ActionListener;
import java.awt.event.FocusEvent;
import java.awt.event.FocusListener;
import java.awt.event.MouseEvent;
import java.awt.event.MouseListener;
import java.io.File;
import java.util.ArrayList;
import java.util.Enumeration;
import java.util.LinkedHashMap;
import java.util.List;

import javax.swing.JFileChooser;
import javax.swing.JFrame;
import javax.swing.JMenuItem;
import javax.swing.JPopupMenu;
import javax.swing.JScrollPane;
import javax.swing.JTree;
import javax.swing.filechooser.FileFilter;
import javax.swing.tree.DefaultMutableTreeNode;
import javax.swing.tree.DefaultTreeModel;
import javax.swing.tree.TreeNode;
import javax.swing.tree.TreePath;
import javax.swing.tree.TreeSelectionModel;

import us.ihmc.simulationconstructionset.commands.WriteDataCommandExecutor;
import us.ihmc.simulationconstructionset.gui.CreatedNewRegistriesListener;
import us.ihmc.simulationconstructionset.gui.EventDispatchThreadHelper;
import us.ihmc.simulationconstructionset.gui.ForcedRepaintPopupMenu;
import us.ihmc.simulationconstructionset.gui.RegistrySettingsChangedListener;
import us.ihmc.simulationconstructionset.util.RegularExpression;
import us.ihmc.simulationconstructionset.util.SimpleFileWriter;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoVariable;

public class NamespaceHierarchyTree extends JScrollPane implements MouseListener, FocusListener, CreatedNewRegistriesListener
{
   private static final long serialVersionUID = -8489413083414697473L;
   private JTree tree;
   private JFileChooser fileChooser = new JFileChooser();
   private DefaultTreeModel model;
   private DefaultMutableTreeNode top;
   private LinkedHashMap<YoRegistry, DefaultMutableTreeNode> registryTreeNodeMap = new LinkedHashMap<>();
   private LinkedHashMap<DefaultMutableTreeNode, YoRegistry> treeNodeRegistryMap = new LinkedHashMap<>();
   private final JFrame frame;
   private final WriteDataCommandExecutor writeDataCommandExecutor;

   private final RegistrySelectedListener registrySelectedListener;

   protected JPopupMenu popupMenu;

   private final YoRegistry root;
   private List<RegistrySettingsChangedListener> registrySettingsChangedListeners = new ArrayList<>();

   private String filterText = "";
   private boolean showOnlyParameters = false;

   public NamespaceHierarchyTree(RegistrySelectedListener namespaceSelectedListener, JFrame frame, WriteDataCommandExecutor writeDataCommandExecutor,
                                 YoRegistry rootRegistry)
   {
      super();

      EventDispatchThreadHelper.checkThatInEventDispatchThread();

      registrySelectedListener = namespaceSelectedListener;
      root = rootRegistry;
      topOfTreeRegistry = root;
      this.frame = frame;
      this.writeDataCommandExecutor = writeDataCommandExecutor;

      top = new DefaultMutableTreeNode(rootRegistry.getNamespace().getName());
      model = new DefaultTreeModel(top);
      tree = new JTree();
      tree.setModel(model);
      tree.setShowsRootHandles(true);
      this.add(tree);
      setViewportView(tree);
      tree.setRootVisible(false);
      tree.getSelectionModel().setSelectionMode(TreeSelectionModel.DISCONTIGUOUS_TREE_SELECTION);
      tree.addMouseListener(this);
      tree.setToggleClickCount(0);

      initPopupMenu();
      setUpTree(rootRegistry);

      tree.setCellRenderer(new NamespaceHierarchyNodeRenderer(treeNodeRegistryMap));
   }

   public void filter(String filterText)
   {
      this.filterText = filterText;
      createdNewRegistries();
   }

   public void filterParameters(boolean showOnlyParameters)
   {
      this.showOnlyParameters = showOnlyParameters;
      createdNewRegistries();
   }

   private boolean needToSetupTree = true;
   private YoRegistry topOfTreeRegistry;

   @Override
   public void paintComponent(Graphics g)
   {
      if (needToSetupTree)
      {
         setUpTree(topOfTreeRegistry);
         needToSetupTree = false;
      }

      super.paintComponent(g);
   }

   private void setUpTree(final YoRegistry currentregistry)
   {
      //TODO: This is incorrect but exo breaks when using invokelater for some reason.
      // Also, reads registry childeren without locks.
      //      EventDispatchThreadHelper.justRun(new Runnable(){
      //
      //         public void run()
      //         {
      EventDispatchThreadHelper.checkThatInEventDispatchThread();

      addNode(currentregistry);

      for (YoRegistry child : currentregistry.getChildren())
      {
         setUpTree(child);
      }
      //         }});
   }

   private void initPopupMenu()
   {
      popupMenu = new ForcedRepaintPopupMenu();
      JMenuItem toggleVaribleSend = new JMenuItem("Toggle Variable Send");
      toggleVaribleSend.addActionListener(new ActionListener()
      {
         @Override
         public void actionPerformed(ActionEvent e)
         {
            toggleVariableSend(e);
            popupMenu.setVisible(false);
         }
      });
      popupMenu.add(toggleVaribleSend);

      JMenuItem recursiveToggleVaribleSend = new JMenuItem("Recursive Toggle Variable Send");
      recursiveToggleVaribleSend.addActionListener(new ActionListener()
      {
         @Override
         public void actionPerformed(ActionEvent e)
         {
            recursiveToggleVariableSend(e);
            popupMenu.setVisible(false);
         }
      });
      popupMenu.add(recursiveToggleVaribleSend);

      JMenuItem toggleVariableLog = new JMenuItem("Toggle Variable Log");
      toggleVariableLog.addActionListener(new ActionListener()
      {
         @Override
         public void actionPerformed(ActionEvent e)
         {
            toggleVariableLog(e);
            popupMenu.setVisible(false);
         }
      });
      popupMenu.add(toggleVariableLog);

      JMenuItem recursiveToggleVariableLog = new JMenuItem("Recursive Toggle Variable Log");
      recursiveToggleVariableLog.addActionListener(new ActionListener()
      {
         @Override
         public void actionPerformed(ActionEvent e)
         {
            recursiveToggleVariableLog(e);
            popupMenu.setVisible(false);
         }
      });
      popupMenu.add(recursiveToggleVariableLog);

      JMenuItem exportBinaryRegistryData = new JMenuItem("Export to binary format");
      exportBinaryRegistryData.addActionListener(new ActionListener()
      {

         @Override
         public void actionPerformed(ActionEvent e)
         {
            exportRegistryData(e, true);
            popupMenu.setVisible(false);
         }
      });
      popupMenu.add(exportBinaryRegistryData);

      JMenuItem exportASCIIRegistryData = new JMenuItem("Export to ASCII format");
      exportASCIIRegistryData.addActionListener(new ActionListener()
      {

         @Override
         public void actionPerformed(ActionEvent e)
         {
            exportRegistryData(e, false);
            popupMenu.setVisible(false);
         }
      });
      popupMenu.add(exportASCIIRegistryData);

      JMenuItem save = new JMenuItem("Save Configuration");
      save.addActionListener(new ActionListener()
      {
         @Override
         public void actionPerformed(ActionEvent e)
         {
            saveConfiguration();
            popupMenu.setVisible(false);
         }
      });
      popupMenu.add(save);

      JMenuItem load = new JMenuItem("Load Configuration");
      load.addActionListener(new ActionListener()
      {
         @Override
         public void actionPerformed(ActionEvent e)
         {
            loadConfiguration();
            popupMenu.setVisible(false);
         }
      });
      popupMenu.add(load);

      popupMenu.addFocusListener(this);

      // tree.addFocusListener(this);
      // this.addFocusListener(this);
   }

   private void addNode(YoRegistry registry)
   {
      EventDispatchThreadHelper.checkThatInEventDispatchThread();

      if (!tree.isExpanded(new TreePath(top.getPath())))
      {
         tree.expandPath(new TreePath(top.getPath()));
      }

      boolean match = RegularExpression.check(registry.getNamespace().getName(), filterText);
      if (showOnlyParameters && match)
      {
         match = registry.hasParametersDeep();
      }

      if (match)
      {
         // create the top of the tree
         if (registry.getParent() == null || !registryTreeNodeMap.containsKey(registry.getParent()))
         {
            DefaultMutableTreeNode newTopLevelAncestor = new DefaultMutableTreeNode(registry.getNamespace().getShortName());
            registryTreeNodeMap.put(registry, newTopLevelAncestor);
            treeNodeRegistryMap.put(newTopLevelAncestor, registry);
            top.add(newTopLevelAncestor);

            model.setRoot(top);
         }
         else
         {
            DefaultMutableTreeNode newChild = new DefaultMutableTreeNode(registry.getNamespace().getShortName());
            registryTreeNodeMap.put(registry, newChild);
            treeNodeRegistryMap.put(newChild, registry);
            registryTreeNodeMap.get(registry.getParent()).add(newChild);
         }
      }
   }

   public void showNamespace(YoRegistry registry)
   {
      if (registryTreeNodeMap.containsKey(registry))
      {
         TreePath pathToNamespace = new TreePath(registryTreeNodeMap.get(registry).getPath());

         tree.expandPath(pathToNamespace);
         tree.scrollPathToVisible(pathToNamespace);
         tree.setSelectionPath(pathToNamespace);
      }
      else
      {
         System.err.println("Warning: " + registry.getNamespace().getName() + " not found.");
      }
   }

   @Override
   public void mouseClicked(MouseEvent arg0)
   {
      if (arg0.getClickCount() == 2)
      {
         TreePath treePath = tree.getPathForLocation(arg0.getX(), arg0.getY());
         if ((treePath != null) && (treePath.getPathCount() >= 1))
         {
            if (registryTreeNodeMap.containsValue(treePath.getLastPathComponent()))
            {
               YoRegistry registry = treeNodeRegistryMap.get(treePath.getLastPathComponent());
               registrySelectedListener.registryWasSelected(registry);
            }
         }
      }

   }

   @Override
   public void mouseEntered(MouseEvent arg0)
   {
   }

   @Override
   public void mouseExited(MouseEvent arg0)
   {
   }

   @Override
   public void mousePressed(MouseEvent arg0)
   {
      if (arg0.isPopupTrigger())
      {
         popupMenu.show((Component) arg0.getSource(), arg0.getX(), arg0.getY());
      }
   }

   @Override
   public void mouseReleased(MouseEvent arg0)
   {
      if (arg0.isPopupTrigger())
      {
         popupMenu.show((Component) arg0.getSource(), arg0.getX(), arg0.getY());
      }
   }

   private void recursiveToggleVariableSend(ActionEvent arg0)
   {
      boolean toggleSend = true;
      boolean toggleLog = false;
      boolean recursive = true;

      toggleVariableSendOrLog(arg0, toggleSend, toggleLog, recursive);
   }

   private void toggleVariableSend(ActionEvent arg0)
   {
      boolean toggleSend = true;
      boolean toggleLog = false;
      boolean recursive = false;

      toggleVariableSendOrLog(arg0, toggleSend, toggleLog, recursive);
   }

   private void recursiveToggleVariableLog(ActionEvent arg0)
   {
      boolean toggleSend = false;
      boolean toggleLog = true;
      boolean recursive = true;

      toggleVariableSendOrLog(arg0, toggleSend, toggleLog, recursive);
   }

   private void toggleVariableLog(ActionEvent arg0)
   {
      boolean toggleSend = false;
      boolean toggleLog = true;
      boolean recursive = false;

      toggleVariableSendOrLog(arg0, toggleSend, toggleLog, recursive);
   }

   private void toggleVariableSendOrLog(ActionEvent arg0, boolean toggleSend, boolean toggleLog, boolean recursive)
   {
      TreePath[] selectionPaths = tree.getSelectionPaths();
      List<YoRegistry> changedRegistries = new ArrayList<>();
      for (int i = 0; i < selectionPaths.length; i++)
      {
         TreePath path = selectionPaths[i];
         TreeNode parentNode = (TreeNode) path.getLastPathComponent();

         YoRegistry parentRegistry = treeNodeRegistryMap.get(parentNode);

         recursiveToggleVariableSendOrLog(recursive, toggleSend, toggleLog, parentNode, parentRegistry, changedRegistries);
      }

      notifyRegistrySettingsChangedListeners(changedRegistries);
   }

   private void exportRegistryData(ActionEvent e, final boolean binary)
   {
      TreePath[] selectionPaths = tree.getSelectionPaths();
      List<YoVariable> allVariables = new ArrayList<>();

      for (int i = 0; i < selectionPaths.length; i++)
      {
         TreePath path = selectionPaths[i];
         TreeNode parentNode = (TreeNode) path.getLastPathComponent();

         YoRegistry parentRegistry = treeNodeRegistryMap.get(parentNode);
         allVariables.addAll(parentRegistry.subtreeVariables());
      }

      FileFilter filterFilter = new FileFilter()
      {

         @Override
         public String getDescription()
         {
            if (binary)
               return ".data.gz";
            else
               return ".m";
         }

         @Override
         public boolean accept(File f)
         {
            if (f.isDirectory())
               return true;

            if (f.getName().toLowerCase().endsWith(getDescription()))
               return true;

            else
               return false;
         }
      };

      fileChooser.setFileFilter(filterFilter);

      if (fileChooser.showSaveDialog(frame) == JFileChooser.APPROVE_OPTION)
      {
         String fileExt = filterFilter.getDescription();
         File chosenFile = fileChooser.getSelectedFile();
         String fileName = chosenFile.getName();

         if (!fileName.endsWith(fileExt))
         {
            fileName = fileName.concat(fileExt);

            if (!chosenFile.getName().equals(fileName))
            {
               chosenFile = new File(chosenFile.getParent(), fileName);
            }
         }

         writeDataCommandExecutor.writeData(allVariables, binary, binary ? true : false, chosenFile);
      }

   }

   private void recursiveToggleVariableSendOrLog(boolean recursive, boolean toggleSend, boolean toggleLog, TreeNode parentNode, YoRegistry parentRegistry,
                                                 List<YoRegistry> changedRegistries)
   {
      changedRegistries.add(parentRegistry);
      model.nodeChanged(parentNode);

      if (recursive)
      {
         Enumeration<? extends TreeNode> childrenNodes = parentNode.children();

         while (childrenNodes.hasMoreElements())
         {
            TreeNode childNode = childrenNodes.nextElement();
            YoRegistry childRegistry = treeNodeRegistryMap.get(childNode);

            recursiveToggleVariableSendOrLog(recursive, toggleSend, toggleLog, childNode, childRegistry, changedRegistries);
         }
      }
   }

   @Override
   public void focusGained(FocusEvent e)
   {
   }

   @Override
   public void focusLost(FocusEvent e)
   {
      //    popupMenu.setVisible(false);
   }

   public void loadConfiguration()
   {
      File configs = new File("Configurations");
      JFileChooser openDialog = new JFileChooser(configs);
      File selectedFile = null;
      if (JFileChooser.APPROVE_OPTION == openDialog.showOpenDialog(openDialog))
      {
         selectedFile = openDialog.getSelectedFile();
      }

      loadConfiguration(selectedFile);
   }

   public void loadConfiguration(File selectedFile)
   {
      if (selectedFile != null)
      {
         if (selectedFile.exists())
         {
            List<YoRegistry> treeNodes = root.subtreeRegistries();

            // getAllRegistrys(rootRegistry, treeNodes);

            // System.out.println(i+" "+realCound+" "+rootRegistry.createVarListsIncludingChildren().size());

            notifyRegistrySettingsChangedListeners();
         }
      }
   }

   private void notifyRegistrySettingsChangedListeners()
   {
      for (RegistrySettingsChangedListener listener : registrySettingsChangedListeners)
      {
         listener.registrySettingsChanged();
      }
   }

   private void notifyRegistrySettingsChangedListeners(List<YoRegistry> yoVariableRegistries)
   {
      for (RegistrySettingsChangedListener listener : registrySettingsChangedListeners)
      {
         listener.registrySettingsChanged(yoVariableRegistries);
      }
   }

   public void saveConfiguration()
   {
      System.out.println("save");
      File configs = new File("Configurations");
      JFileChooser saveDialog = new JFileChooser(configs);
      File selectedFile = null;
      if (JFileChooser.APPROVE_OPTION == saveDialog.showSaveDialog(this))
      {
         selectedFile = saveDialog.getSelectedFile();
      }

      if (selectedFile != null)
      {
         SimpleFileWriter writer = new SimpleFileWriter(selectedFile);
         List<YoRegistry> treeNodes = root.subtreeRegistries();

         String outString = "";
         for (YoRegistry node : treeNodes)
         {
            outString += node.getNamespace().getName() + "\n";
         }

         writer.write(outString);
         writer.close();
      }
   }

   @Override
   public void createdNewRegistries()
   {
      top.removeAllChildren();
      registryTreeNodeMap.clear();
      treeNodeRegistryMap.clear();

      topOfTreeRegistry = root;
      needToSetupTree = true;
      //      setUpTree(root);
   }

   public void addRegistrySettingsChangedListener(RegistrySettingsChangedListener listener)
   {
      registrySettingsChangedListeners.add(listener);
   }
}
