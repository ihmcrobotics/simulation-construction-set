---
title: Data Buffer
---

As the robot is simulated, the data is stored in a data buffer. The period in which data is stored defaults to once every 20 milliseconds (50 Hertz). This can be changed through the [SimulationConstructionSet API](https://ihmcrobotics.github.io/simulation-construction-set/docs/00-scs-initialization). The default size of the data buffer is 16000 points. With these two defaults, 320 seconds of data can be recorded. If the simulation runs over this size, the current index is wrapped to the beginning of the buffer.

To change the data buffer properties, go to `Data Buffer->Data Buffer Properties` on the menu.

![DataBuffer](/img/scs-tutorials/scsDataBufferProperties.png)

In the Data Buffer Properties dialog you can change the current size of the buffer.  You can also change the default handling of the buffer to Enlarge instead of Wrap.  When in Enlarge mode, the buffer will increase in size by 25% each time the end is reached until the buffer reaches `Max Size`.

You can also change the buffer size by cropping it to the in point and out point.  To do this, choose a window of data that you'd like to crop and set the in point to the beginning of the window and the out point to the end of the window. Then go to `Data Buffer->Crop Buffer to In/Out` in the menu. The data should be cropped to the window you selected.

Sometimes you may wish to move the data so that the in point is the first index but you don't wish to crop the data.  To do this, go to `Data Buffer->Pack Buffer to In/Out` on the menu. This will rotate the data in the buffer so that the in point is "packed" to the left.
