---
title: Robot
---

A Robot consists of one or more trees of Joints, with each Joint having a corresponding link with mass and inertia. 
For example, if you are creating a bipedal robot, the first Joint might be a FloatingJoint (6 degree of freedom joint) with its link set to be the torso of the robot.
Children of the FloatingJoint would then be the 2 hips with Links set to be the upper legs, and two shoulder joints, with Links being the upper arms.
The structure would continue in that fashion with knees being children of the hips, ankles being children of the knees, etc. 