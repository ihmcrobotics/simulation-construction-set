package us.ihmc.simulationconstructionset.gui;

import java.util.List;

import us.ihmc.yoVariables.registry.YoVariableRegistry;

public interface RegistrySettingsChangedListener
{
   public abstract void registrySettingsChanged(List<YoVariableRegistry> changedRegistries);

   public abstract void registrySettingsChanged();
}
