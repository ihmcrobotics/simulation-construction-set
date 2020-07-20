package us.ihmc.simulationconstructionset.commands;

import us.ihmc.simulationconstructionset.GotoInPointCommandExecutor;
import us.ihmc.simulationconstructionset.GotoOutPointCommandExecutor;
import us.ihmc.simulationconstructionset.gui.dialogConstructors.GUIEnablerAndDisabler;
import us.ihmc.yoVariables.buffer.interfaces.KeyPointsHolder;
import us.ihmc.yoVariables.buffer.interfaces.YoBufferReader;

public interface AllCommandsExecutor extends YoBufferReader, RunCommandsExecutor, AddCameraKeyCommandExecutor, AddKeyPointCommandExecutor,
      CreateNewGraphWindowCommandExecutor, CreateNewViewportWindowCommandExecutor, CropBufferCommandExecutor, PackBufferCommandExecutor,
      CutBufferCommandExecutor, ThinBufferCommandExecutor, NextCameraKeyCommandExecutor, PreviousCameraKeyCommandExecutor, RemoveCameraKeyCommandExecutor,
      SelectGUIConfigFromFileCommandExecutor, SetInPointCommandExecutor, SetOutPointCommandExecutor, StepBackwardCommandExecutor, StepForwardCommandExecutor,
      ToggleCameraKeyModeCommandExecutor, KeyPointsHolder, ViewportSelectorCommandExecutor, ZoomGraphCommandExecutor,
      ExportSnapshotCommandExecutor, GUIEnablerAndDisabler, CreateNewYoVariableSliderWindowCommandExecutor, GotoInPointCommandExecutor, GotoOutPointCommandExecutor
{
}
