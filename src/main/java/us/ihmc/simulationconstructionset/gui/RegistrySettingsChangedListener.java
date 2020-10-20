package us.ihmc.simulationconstructionset.gui;

import java.util.List;

import us.ihmc.yoVariables.registry.YoRegistry;

public interface RegistrySettingsChangedListener
{
   public abstract void registrySettingsChanged(List<YoRegistry> changedRegistries);

   public abstract void registrySettingsChanged();
}
