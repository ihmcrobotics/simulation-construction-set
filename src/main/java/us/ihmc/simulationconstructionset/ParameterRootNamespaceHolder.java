package us.ihmc.simulationconstructionset;

import java.io.File;

import us.ihmc.yoVariables.registry.YoNamespace;

public interface ParameterRootNamespaceHolder
{
   public YoNamespace getParameterRootPath();

   public File getDefaultParameterFile();
}
