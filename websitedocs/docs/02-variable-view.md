---
title: SCS Variables
---

In the variable view on the bottom left you can view the current value of any simulation variable and change its value as you did with the `walk` variable in the [Quick Start](https://ihmcrobotics.github.io/ihmc-open-robotics-software/docs/quickstarthome.html).  This view contains three tabs: Name Space, Variables, and Search.

### Name Space Tab

Select the `Name Space` tab at the top of the Variables view.  You will learn more about name spaces when you create a sim from scratch, but for now suffice it to say that a name space is a way to organize variables in a tree structure.  If you expand the root node and then the valkyrie node, you will see the following:

![NameSpaceView](/img/scs-tutorials/svsVariablesNameSpace.png)

Double-click on the valkyrie node.  This will change the `Variables` tab title to `Valkyrie`.

### Variables Tab

![VariableView](/img/scs-tutorials/svsVariablesVariableSpace.png)

This tab shows the variables that are in the name space which was double-clicked on in the `Name Space` tab.  Select the variable `q_z`.  It will change to be red and also show up at the bottom of the view.  You can modify it's value in a few ways.  The first is by clicking on the arrows to the right and left of its value which will increment or decrement the value.  Otherwise you can double-click on the value edit it, pressing the `Enter` key or `Tab` key for the change to take affect.  If you modify the `q_z` value you should see the Valkyrie robot move up or down in the 3D view.  If you try and run the simulation after changing a value and it crashes, remember that you will need to restart inorder to correct the crash.

### Search Tab

The most used tab is the `Search` tab.  This is the tab which is visible by default when the simulation starts.  In the Search field you can enter text and any variable that  matches will show up in the results box.  Type `q_left` and you will see many of the Valkyrie robot's left-side joint position values.  If you would like to see a variable all the time you can drag it from the results view to the area right below, and it will remain there even when you change the search text.  You can modify values here the same as you did in the previous view.  To remove a variable from this persistent view, right-click and select `Remove Variable`.

![SearchView](/img/scs-tutorials/svsVariablesSearchSpace.png) ![LeftHipPitchView](/img/scs-tutorials/scsLeftHipPitchChanged.png)

