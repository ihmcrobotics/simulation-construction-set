package us.ihmc.simulationconstructionset.gui.actions.dialogActions;

import java.awt.Image;
import java.awt.image.BufferedImage;
import java.io.IOException;
import java.io.InputStream;

import javax.imageio.ImageIO;
import javax.swing.AbstractAction;
import javax.swing.Action;
import javax.swing.ImageIcon;

import us.ihmc.log.LogTools;

public class AbstractActionTools
{
   public static void setupIconButton(AbstractAction action, String iconFilename, int shortKey, String longDescription, String shortDescription)
   {
      // Note: Either URL or InputStream method should work. But make sure to not do .. to go up a directory.
      // That used to work, but doesn't seem to work anymore. Instead, make sure that the resources are at this
      // level of the directory or lower...

      Image image = loadActionImageUsingInputStream(action, iconFilename);

      if (image != null)
      {
         action.putValue(Action.SMALL_ICON, new ImageIcon(image));
      }

      action.putValue(Action.MNEMONIC_KEY, new Integer(shortKey));
      action.putValue(Action.LONG_DESCRIPTION, longDescription);
      action.putValue(Action.SHORT_DESCRIPTION, shortDescription);
   }

   public static Image loadActionImageUsingInputStream(AbstractAction action, String iconFilename)
   {
      InputStream iconInputStream = action.getClass().getClassLoader().getResourceAsStream(iconFilename);
      if (iconInputStream == null)
      {
         System.err.println("Warning! Can't find resource " + iconFilename);
         return null;
      }

      try
      {
         BufferedImage bimage = ImageIO.read(iconInputStream);
         if (bimage.getWidth() != bimage.getHeight())
         {
            LogTools.warn("Icon not square: {} {}x{}", iconFilename, bimage.getWidth(), bimage.getHeight());
         }

         return bimage.getScaledInstance(32, 32, Image.SCALE_DEFAULT);
      }
      catch (IOException e)
      {
         System.err.println("Can't find icon image " + iconFilename);
         return null;
      }
   }
}
