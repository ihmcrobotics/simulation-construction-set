---
title: Simulation Replay
---

Stop the Simulation if it is currently running.

Look at the graphs of the data. Notice that there is a green bar at the beginning of the data, a red bar at the end of the data, and a black bar somewhere in the middle. 

![VarGraph](/img/scs-tutorials/scsScrubGraph.png)

* The green bar marks the data in point.
* The red bar marks the data out point.
* And the black bar marks the current index. 

The in and out points are used in all of the functions which require a window on the data. For example, when you playback the data, the playback loops to the in point once it reaches the out point.

Zoom out so that you can see the in point (green line), out point (red line) and current index (black line) in the data. 

* You can move the sim to the in point by either going to `Run->Goto In Point` on the menu or by pushing the Goto In Point button ![GotoIn](/img/scs-tutorials/scsGotoInPointButton.png).
* Move to the out point by either going to `Run->Goto Out Point` on the menu or by pushing the Goto Out Point button ![GotoOut](/img/scs-tutorials/scsGotoOutPointButton.png).
* Change the in point to current index (black line) by either going to `Run->Set In Point` on the menu or by pushing the Set In Point button ![SetIn](/img/scs-tutorials/scsSetInPointButton.png).
* Change the out point to the current index by going to `Run->Set Out Point` on the menu or by pushing the Set Out Point button ![SetOut](/img/scs-tutorials/scsSetOutPointButton.png).

Play the simulation `Run->Play` on menu or play button: ![replay](/img/scs-tutorials/scsReplayButton.png) and notice that it now loops around the new in and out points. Note that if the in point is after the out point, then the playback loops around the entire data buffer, which is a circular buffer.