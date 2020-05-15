package us.ihmc.simulationconstructionset.commands;

import java.io.File;
import java.util.List;

import us.ihmc.yoVariables.variable.YoVariable;

public interface WriteDataCommandExecutor
{
   public abstract void writeData(List<YoVariable<?>> vars, boolean binary, boolean compress, File chosenFile);
}
