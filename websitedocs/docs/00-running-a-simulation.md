---
title: Running a simulation
---
This tutorial assumes that you have followed the Quick Start guide and have the ValkyrieDemo Simulation up and running.  If you have not, please do so before you continue.  You should now have the Simulation Construction Set (SCS) graphical user interface (GUI) up as shown in Figure 1.  Now that you are ready, let's try running a simulation. The following tutorial will take you through some of the features of SCS's GUI which provides a 3D graphical image of the Simulation, and allows for the changing of variables using numerical entry boxes, real-time plotting of variables, exporting and importing of data, and the creation of jpeg snapshots and MP4 movies.

<a name="figure1"></a>![SCSGUI](/img/scs-tutorials/scsGUI.png)

* **Figure 1: Screenshot of SCS GUI with the Valkyrie Humanoid Robot**

1. Look around and familiarize yourself with the interface. A menu bar is on the top of the screen. A 3D graphical window showing the robot is below that. To the right of the 3D window is a plotter showing an overhead view of balance data. Below the 3D window is a toolbar with buttons for the most common functions. On the bottom left are variable panels containing lists of all the simulation and control variables. To the right of the variable panels are graph windows for graphing any variable. On the bottom are numeric entry boxes for changing the value of variables.
2. Start the simulation by either going to `Run->Simulate` from the menu bar or by pressing the Simulate button ![SimulateButton](/img/scs-tutorials/scsSimulateButton.png) on the toolbar. The robot will stand in place or walk depending on the value of the `walk` variable (0.0 = stand, 1 = walk) and the variable values in the variable panels will update.
3. After letting the simulation run for a few seconds, stop it by either going to `Run->Stop` on the menu bar or by pushing the stop button ![Stop](/img/scs-tutorials/scsStopButton.png). The simulation should now stop.
4. Replay the simulated data by either going to `Run->Play` on the menu or by pushing the play button ![Replay](/img/scs-tutorials/scsReplayButton.png) . The simulated data will be played back in the 3D graphics window in real time.

* Note that if you replay the Valkyrie simulation, in order to simulate again without restarting, choose `Run->Goto Out Point` from the menu or press the Goto Out Point toolbar button ![GotoOut](/img/scs-tutorials/scsGotoOutPointButton.png).
