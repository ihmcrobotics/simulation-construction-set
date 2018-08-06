---
title: Description and Analysis
---

### 1. Add the constructor to the FallingBrickRobot class
```java
  public FallingBrickRobot()
     {
        super("FallingBrick");
         
        this.setGravity(0.0, 0.0, -G);
        // create the brick as a floating joint
        floatingJoint = new FloatingJoint("base", new Vector3d(0.0, 0.0, 0.0), this);
        Link link1 = base("base", YoAppearance.Red());
        floatingJoint.setLink(link1);
        this.addRootJoint(floatingJoint);
        // add ground contact points to the brick
        GroundContactPoint gc1 = new GroundContactPoint("gc1", new Vector3d(BASE_L / 2.0, BASE_W / 2.0, BASE_H / 2.0), this);
        floatingJoint.addGroundContactPoint(gc1);
        GroundContactPoint gc2 = new GroundContactPoint("gc2", new Vector3d(BASE_L / 2.0, -BASE_W / 2.0, BASE_H / 2.0), this);
        floatingJoint.addGroundContactPoint(gc2);
        GroundContactPoint gc3 = new GroundContactPoint("gc3", new Vector3d(-BASE_L / 2.0, BASE_W / 2.0, BASE_H / 2.0), this);
        floatingJoint.addGroundContactPoint(gc3);
        GroundContactPoint gc4 = new GroundContactPoint("gc4", new Vector3d(-BASE_L / 2.0, -BASE_W / 2.0, BASE_H / 2.0), this);
        floatingJoint.addGroundContactPoint(gc4);
        GroundContactPoint gc5 = new GroundContactPoint("gc5", new Vector3d(BASE_L / 2.0, BASE_W / 2.0, -BASE_H / 2.0), this);
        floatingJoint.addGroundContactPoint(gc5);
        GroundContactPoint gc6 = new GroundContactPoint("gc6", new Vector3d(BASE_L / 2.0, -BASE_W / 2.0, -BASE_H / 2.0), this);
        floatingJoint.addGroundContactPoint(gc6);
        GroundContactPoint gc7 = new GroundContactPoint("gc7", new Vector3d(-BASE_L / 2.0, BASE_W / 2.0, -BASE_H / 2.0), this);
        floatingJoint.addGroundContactPoint(gc7);
        GroundContactPoint gc8 = new GroundContactPoint("gc8", new Vector3d(-BASE_L / 2.0, -BASE_W / 2.0, -BASE_H / 2.0), this);
        floatingJoint.addGroundContactPoint(gc8);
        GroundContactPoint gc9 = new GroundContactPoint("gc9", new Vector3d(0.0, 0.0, BASE_H / 2.0 + BASE_H), this);
        floatingJoint.addGroundContactPoint(gc9);
        GroundContactPoint gc10 = new GroundContactPoint("gc10", new Vector3d(0.0, 0.0, -BASE_H / 2.0 - BASE_H), this);
        floatingJoint.addGroundContactPoint(gc10);
        this.setController(this); // tells the simulator to call the local doControl() method
        // instantiate ground contact model
        GroundContactModel groundModel = new LinearGroundContactModel(this, 1422, 150.6, 50.0, 1000.0,
                                                                      this.getRobotsYoVariableRegistry());
        // GroundContactModel groundModel = new CollisionGroundContactModel(this, 0.5, 0.7);
        GroundProfile3D profile = new WavyGroundProfile();
        groundModel.setGroundProfile3D(profile);
        this.setGroundContactModel(groundModel);
        initRobot();
        initControl();
     }
     /**
      * This method returns a brick link instance.
      */
     private Link base(String name, AppearanceDefinition appearance)
     {
        Link ret = new Link(name);
        ret.setMass(M1);
        ret.setMomentOfInertia(Ixx1, Iyy1, Izz1);
        ret.setComOffset(0.0, 0.0, 0.0);
        LinkGraphicsDescription linkGraphics = new LinkGraphicsDescription();
        linkGraphics.translate(0.0, 0.0, -B1);
        // linkGraphics.addCube((float)BASE_L, (float)BASE_W, (float)BASE_H, appearance);
        // linkGraphics.addCone((float)BASE_L,(float)BASE_W);
        linkGraphics.addPyramidCube(BASE_L, BASE_W, BASE_H, BASE_H, appearance);
        ret.setLinkGraphics(linkGraphics);
        return ret;
     }
     
     
     public YoVariableRegistry getYoVariableRegistry()
     {
        return registry;
     }
      
     public void initialize()
     {     
     }
     public String getDescription()
     {
        return getName();
     }
  }
  ```

Find the following 4 lines in `FallingBrickRobot` where the ground contact is defined:

```java
GroundContactModel groundModel = new LinearGroundContactModel(this, 1422, 150.6, 50.0, 1000.0, this.getRobotsYoVariableRegistry());
GroundProfile3D profile = new WavyGroundProfile();
groundModel.setGroundProfile3D(profile);
this.setGroundContactModel(groundModel);
```

Lets look at it one line at a time.  
   Notice that the `groundModel` is an instance of the class `LinearGroundContactModel` and the `GroundProfile` is an instance of the class `WavyGroundProfile()`. 

### 2. Next add the method initRobot method in the FallingBrickRobot class

This method simply sets the bricks position, velocity, and acceleration initially.

```java
     public void initRobot()
     {
        q_qlength = new DoubleYoVariable("q_qlength", registry);
        theta_x = new DoubleYoVariable("theta_x", registry);
        t.set(0.0);
        q_x = (DoubleYoVariable)this.getVariable("q_x");
        q_y = (DoubleYoVariable)this.getVariable("q_y");
        q_z = (DoubleYoVariable)this.getVariable("q_z");
        qd_x = (DoubleYoVariable)this.getVariable("qd_x");
        qd_y = (DoubleYoVariable)this.getVariable("qd_y");
        qd_z = (DoubleYoVariable)this.getVariable("qd_z");
        qdd_x = (DoubleYoVariable)this.getVariable("qdd_x");
        qdd_y = (DoubleYoVariable)this.getVariable("qdd_y");
        qdd_z = (DoubleYoVariable)this.getVariable("qdd_z");
        q_qs = (DoubleYoVariable)this.getVariable("q_qs");
        q_qx = (DoubleYoVariable)this.getVariable("q_qx");
        q_qy = (DoubleYoVariable)this.getVariable("q_qy");
        q_qz = (DoubleYoVariable)this.getVariable("q_qz");
        qd_wx = (DoubleYoVariable)this.getVariable("qd_wx");
        qd_wy = (DoubleYoVariable)this.getVariable("qd_wy");
        qd_wz = (DoubleYoVariable)this.getVariable("qd_wz");
        qdd_wx = (DoubleYoVariable)this.getVariable("qdd_wx");
        qdd_wy = (DoubleYoVariable)this.getVariable("qdd_wy");
        qdd_wz = (DoubleYoVariable)this.getVariable("qdd_wz");
        q_x.set(0.0);
        q_y.set(0.0);
        q_z.set(0.6);
        qd_x.set(0.0);
        qd_y.set(0.0);
        qd_z.set(0.0);
        q_qs.set(0.707);
        q_qx.set(0.3);
        q_qy.set(0.4);
        q_qz.set(0.5);
        qd_wx.set(0.0001);
        qd_wy.set(1.0);
        qd_wz.set(0.5001);
     }
```


### 3. Next add the initControl method to the FallingBrickRobot class
   This method simply initializes the robots controller.

```java
 public void initControl()
     {
        qdd2_wx = new DoubleYoVariable("qdd2_wx", registry);
        qdd2_wy = new DoubleYoVariable("qdd2_wy", registry);
        qdd2_wz = new DoubleYoVariable("qdd2_wz", registry);
        energy = new DoubleYoVariable("energy", registry);
     }
     
     public void doControl()
          {
             energy.set(  M1 * G * q_z.getDoubleValue() 
                          + 0.5 * M1 * qd_x.getDoubleValue() * qd_x.getDoubleValue() 
                          + 0.5 * M1 * qd_y.getDoubleValue() * qd_y.getDoubleValue() 
                          + 0.5 * M1 * qd_z.getDoubleValue() * qd_z.getDoubleValue()
                          + 0.5 * Ixx1 * qd_wx.getDoubleValue() * qd_wx.getDoubleValue() 
                          + 0.5 * Iyy1 * qd_wy.getDoubleValue() * qd_wy.getDoubleValue()
                          + 0.5 * Izz1 * qd_wz.getDoubleValue() * qd_wz.getDoubleValue());
                          
             qdd2_wx.set((Iyy1 - Izz1) / Ixx1 * qd_wy.getDoubleValue() * qd_wz.getDoubleValue());
             qdd2_wy.set((Izz1 - Ixx1) / Iyy1 * qd_wz.getDoubleValue() * qd_wx.getDoubleValue());
             qdd2_wz.set((Ixx1 - Iyy1) / Izz1 * qd_wx.getDoubleValue() * qd_wy.getDoubleValue());
          }
```

### 4. Add methods to WavyGroundProfile class
   
   Add the following methods to your WavyGroundProfile class:
   
```java
      public double heightAt(double x, double y, double z)
      {
         if ((x > xMin) && (x < xMax) && (y > yMin) && (y < yMax))
            return 1.0 * Math.exp(-Math.abs(2.0 * x)) * Math.exp(-Math.abs(2.0 * y)) * Math.sin(2.0 * Math.PI * 0.7 * x);
         else
            return 0.0;
      }
      public void surfaceNormalAt(double x, double y, double z, Vector3d normal)
      {
         normal.x = 0.0;
         normal.y = 0.0;
         normal.z = 1.0;
      }
      public void closestIntersectionTo(double x, double y, double z, Point3d point)
      {
         point.x = x;
         point.y = y;
         point.z = heightAt(x, y, z);
      }
      public void closestIntersectionAndNormalAt(double x, double y, double z, Point3d point, Vector3d normal)
      {
         closestIntersectionTo(x, y, z, point);
         surfaceNormalAt(x, y, z, normal);
      }
      public boolean checkIfInside(double x, double y, double z, Point3d intersectionToPack, Vector3d normalToPack)
      {
         closestIntersectionTo(x, y, z, intersectionToPack);
         surfaceNormalAt(x, y, z, normalToPack);
          
         return (z < intersectionToPack.getZ());
      }
       
      public boolean isClose(double x, double y, double z)
      {
         return boundingBox.isInside(x, y, z);
      }
      public BoundingBox3d getBoundingBox()
      {
         return boundingBox;
      }
```
   
Since `WavyGroundProfile` implements the interface `GroundProfile`, it defines the methods:  
   `heightAt`, `isClose`, `surfaceNormalAt`, `closestIntersectionTo`, `closestIntersectionAndNormalAt`, `getXmin`, `getXmax`, `getYmin`, `getYmax`, `getXTiles`, and `getYTiles`. 
   
   The point where the height of the ground is defined is in heightAt:  
   **return 1.0 * Math.exp(-Math.abs(2.0*x)) * Math.exp(-Math.abs(2.0*y)) * Math.sin(2.0*Math.PI*0.7*x);**

### 5. Change the profile of the terrain
   Do this by changing the function `heightAt`. Run the simulation and see how the profile of the ground changed.
   
   To read more about how to use and setup GroundProfile check out the [Ground Profile Interface API](https://ihmcrobotics.github.io/simulation-construction-set/docs/01-introduction-to-points.html) page.

### 6. Examine the file LinearGroundContactModel.java
   Since `LinearGroundContactModel` implements the interface `GroundContactModel`, it defines the method `doGroundContact`, where the ground contact forces are computed.

### 7. Study the doGroundContact and resolveContactForceZUp methods
   These can be found in the file LinearStickSlipGroundContactModel.java  
   
   Notice that the ground is modeled as a linear spring-damper in the x and y directions and a non-linear spring and a linear damper in the z direction.
   Using a non-linear (hardening) spring in the z direction is a standard way to prevent ground chatter or bounce while still simulating a stiff ground. This `GroundContactModel` is a simple one and does not take into consideration the surface normal of the ground, or ground slipping. 
   For this example simulation, that is ok, but in many instances, these effects are important. If so, then you should use one of the other ground contact models in the `com.yobotics.simulationconstructionset.utils` package, or create your own.

### 8. Experiment with different values of ground spring and damping constants
   Note that as you lower the stiffnesses, greater penetration into the ground will occur. As you lower the damping constants, vibrations occur longer. 
   However, if you increase the stiffness or damping constants too much, instabilities can occur due to numerical instabilities.

### 9. As a general rule of thumb
   Ground stiffness and damping are tuned experimentally until an acceptable ground penetration and bounce is achieved. 
   Try tuning the ground parameters for different ground penetration and bounce.


Various GroundProfiles and GroundContactModels are provided in the `com.yobotics.simulationconstructionset.util.ground` directory.
You should examine the source code of these files (found in SimulationConstructionSet\src) 
in order to determine the model and profile best for your simulation, and to understand how to create your own custom ground contact model.

### Full Code for Classes
<details>
<summary>FallingBrickSimulation</summary>

```java
package us.ihmc.exampleSimulations.fallingBrick  ;
import us.ihmc.graphicsDescription.appearance.YoAppearance;
import us.ihmc.simulationconstructionset.SimulationConstructionSet;
import us.ihmc.simulationconstructionset.SimulationConstructionSetParameters;
import us.ihmc.simulationconstructionset.examples.FallingBrickRobot;
public class FallingBrickSimulation
{
    private enum GROUND_APPEARANCE
    {
        EARTH, STONE, ALUMINUM
    }
    SimulationConstructionSet sim;
    public FallingBrickSimulation()
    {
        GROUND_APPEARANCE appearance = GROUND_APPEARANCE.EARTH;
        FallingBrickRobot fallingBrick = new FallingBrickRobot();
        sim = new SimulationConstructionSet(fallingBrick, new SimulationConstructionSetParameters(16342));
        sim.setDT(0.001, 20);
        sim.setCameraPosition(-1.5, -2.5, 0.5);
        sim.setCameraFix(0.0, 0.0, 0.4);
        sim.setCameraTracking(false, true, true, false);
        sim.setCameraDolly(false, true, true, false);
        // Set up some graphs:
        sim.setupGraph("q_z");
        sim.setupEntryBox("qd_x");
        sim.setupEntryBox("qd_y");
        sim.setupEntryBox("qd_z");
        sim.setupEntryBox("qd_wx");
        sim.setupEntryBox("qd_wy");
        sim.setupEntryBox("qd_wz");
        switch (appearance)
        {
            case EARTH:
                sim.setGroundAppearance(YoAppearance.EarthTexture());
                break;
            case STONE:
                sim.setGroundAppearance(YoAppearance.StoneTexture());
                break;
            case ALUMINUM:
                sim.setGroundAppearance(YoAppearance.AluminumMaterial());
                break;
        }
        Thread myThread = new Thread(sim);
        myThread.start();
    }
    public static void main(String[] args)
    {
        new FallingBrickSimulation();
    }
}
```
</details>

<details>
<summary>FallingBrickRobot</summary>

```java
package us.ihmc.exampleSimulations.fallingBrick;
import javax.vecmath.Vector3d;
import us.ihmc.jMonkeyEngineToolkit.GroundProfile3D;
import us.ihmc.robotics.robotDescription.LinkGraphicsDescription;
import us.ihmc.graphicsDescription.appearance.AppearanceDefinition;
import us.ihmc.graphicsDescription.appearance.YoAppearance;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.simulationconstructionset.FloatingJoint;
import us.ihmc.simulationconstructionset.GroundContactModel;
import us.ihmc.simulationconstructionset.GroundContactPoint;
import us.ihmc.simulationconstructionset.Joint;
import us.ihmc.simulationconstructionset.Link;
import us.ihmc.simulationconstructionset.Robot;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.robotController.RobotController;
import us.ihmc.simulationconstructionset.util.LinearGroundContactModel;
import us.ihmc.simulationconstructionset.util.ground.WavyGroundProfile;
public class FallingBrickRobot extends Robot implements RobotController
{
    private static final long serialVersionUID = 773713164696806099L;

    private static final double BASE_H = 0.1, BASE_W = 0.2, BASE_L = 0.3;
    private static final double B1 = BASE_H / 2.0;
    private static final double M1 = 1.7;
    private static final double Ixx1 = 0.1, Iyy1 = 0.5, Izz1 = 0.9;
    private static final double G = 9.81;
    private final YoVariableRegistry registry = new YoVariableRegistry("FallingBrickController");
    // position, velocity, and acceleration variables
    DoubleYoVariable q_x, q_y, q_z, qd_x, qd_y, qd_z, qdd_x, qdd_y, qdd_z;
    DoubleYoVariable q_qs, q_qx, q_qy, q_qz, qd_wx, qd_wy, qd_wz, qdd_wx, qdd_wy, qdd_wz;
    DoubleYoVariable energy, q_qlength, theta_x;
    DoubleYoVariable qdd2_wx, qdd2_wy, qdd2_wz;
    Joint floatingJoint;

    public FallingBrickRobot()
    {
        super("FallingBrick");

        this.setGravity(0.0, 0.0, -G);
        // create the brick as a floating joint
        floatingJoint = new FloatingJoint("base", new Vector3d(0.0, 0.0, 0.0), this);
        Link link1 = base("base", YoAppearance.Red());
        floatingJoint.setLink(link1);
        this.addRootJoint(floatingJoint);
        // add ground contact points to the brick
        GroundContactPoint gc1 = new GroundContactPoint("gc1", new Vector3d(BASE_L / 2.0, BASE_W / 2.0, BASE_H / 2.0), this);
        floatingJoint.addGroundContactPoint(gc1);
        GroundContactPoint gc2 = new GroundContactPoint("gc2", new Vector3d(BASE_L / 2.0, -BASE_W / 2.0, BASE_H / 2.0), this);
        floatingJoint.addGroundContactPoint(gc2);
        GroundContactPoint gc3 = new GroundContactPoint("gc3", new Vector3d(-BASE_L / 2.0, BASE_W / 2.0, BASE_H / 2.0), this);
        floatingJoint.addGroundContactPoint(gc3);
        GroundContactPoint gc4 = new GroundContactPoint("gc4", new Vector3d(-BASE_L / 2.0, -BASE_W / 2.0, BASE_H / 2.0), this);
        floatingJoint.addGroundContactPoint(gc4);
        GroundContactPoint gc5 = new GroundContactPoint("gc5", new Vector3d(BASE_L / 2.0, BASE_W / 2.0, -BASE_H / 2.0), this);
        floatingJoint.addGroundContactPoint(gc5);
        GroundContactPoint gc6 = new GroundContactPoint("gc6", new Vector3d(BASE_L / 2.0, -BASE_W / 2.0, -BASE_H / 2.0), this);
        floatingJoint.addGroundContactPoint(gc6);
        GroundContactPoint gc7 = new GroundContactPoint("gc7", new Vector3d(-BASE_L / 2.0, BASE_W / 2.0, -BASE_H / 2.0), this);
        floatingJoint.addGroundContactPoint(gc7);
        GroundContactPoint gc8 = new GroundContactPoint("gc8", new Vector3d(-BASE_L / 2.0, -BASE_W / 2.0, -BASE_H / 2.0), this);
        floatingJoint.addGroundContactPoint(gc8);
        GroundContactPoint gc9 = new GroundContactPoint("gc9", new Vector3d(0.0, 0.0, BASE_H / 2.0 + BASE_H), this);
        floatingJoint.addGroundContactPoint(gc9);
        GroundContactPoint gc10 = new GroundContactPoint("gc10", new Vector3d(0.0, 0.0, -BASE_H / 2.0 - BASE_H), this);
        floatingJoint.addGroundContactPoint(gc10);
        this.setController(this); // tells the simulator to call the local doControl() method
        // instantiate ground contact model
        GroundContactModel groundModel = new LinearGroundContactModel(this, 1422, 150.6, 50.0, 1000.0,
                this.getRobotsYoVariableRegistry());
        // GroundContactModel groundModel = new CollisionGroundContactModel(this, 0.5, 0.7);
        GroundProfile3D profile = new WavyGroundProfile();
        groundModel.setGroundProfile3D(profile);
        this.setGroundContactModel(groundModel);
        initRobot();
        initControl();
    }
    /**
     * This method returns a brick link instance.
     */
    private Link base(String name, AppearanceDefinition appearance)
    {
        Link ret = new Link(name);
        ret.setMass(M1);
        ret.setMomentOfInertia(Ixx1, Iyy1, Izz1);
        ret.setComOffset(0.0, 0.0, 0.0);
        LinkGraphicsDescription linkGraphics = new LinkGraphicsDescription();
        linkGraphics.translate(0.0, 0.0, -B1);
        // linkGraphics.addCube((float)BASE_L, (float)BASE_W, (float)BASE_H, appearance);
        // linkGraphics.addCone((float)BASE_L,(float)BASE_W);
        linkGraphics.addPyramidCube(BASE_L, BASE_W, BASE_H, BASE_H, appearance);
        ret.setLinkGraphics(linkGraphics);
        return ret;
    }

    /**
     * This method sets the initial positions, velocities, and accelerations of the brick
     */
    public void initRobot()
    {
        q_qlength = new DoubleYoVariable("q_qlength", registry);
        theta_x = new DoubleYoVariable("theta_x", registry);
        t.set(0.0);
        q_x = (DoubleYoVariable)this.getVariable("q_x");
        q_y = (DoubleYoVariable)this.getVariable("q_y");
        q_z = (DoubleYoVariable)this.getVariable("q_z");
        qd_x = (DoubleYoVariable)this.getVariable("qd_x");
        qd_y = (DoubleYoVariable)this.getVariable("qd_y");
        qd_z = (DoubleYoVariable)this.getVariable("qd_z");
        qdd_x = (DoubleYoVariable)this.getVariable("qdd_x");
        qdd_y = (DoubleYoVariable)this.getVariable("qdd_y");
        qdd_z = (DoubleYoVariable)this.getVariable("qdd_z");
        q_qs = (DoubleYoVariable)this.getVariable("q_qs");
        q_qx = (DoubleYoVariable)this.getVariable("q_qx");
        q_qy = (DoubleYoVariable)this.getVariable("q_qy");
        q_qz = (DoubleYoVariable)this.getVariable("q_qz");
        qd_wx = (DoubleYoVariable)this.getVariable("qd_wx");
        qd_wy = (DoubleYoVariable)this.getVariable("qd_wy");
        qd_wz = (DoubleYoVariable)this.getVariable("qd_wz");
        qdd_wx = (DoubleYoVariable)this.getVariable("qdd_wx");
        qdd_wy = (DoubleYoVariable)this.getVariable("qdd_wy");
        qdd_wz = (DoubleYoVariable)this.getVariable("qdd_wz");
        q_x.set(0.0);
        q_y.set(0.0);
        q_z.set(0.6);
        qd_x.set(0.0);
        qd_y.set(0.0);
        qd_z.set(0.0);
        q_qs.set(0.707);
        q_qx.set(0.3);
        q_qy.set(0.4);
        q_qz.set(0.5);
        qd_wx.set(0.0001);
        qd_wy.set(1.0);
        qd_wz.set(0.5001);
    }
    public void initControl()
    {
        qdd2_wx = new DoubleYoVariable("qdd2_wx", registry);
        qdd2_wy = new DoubleYoVariable("qdd2_wy", registry);
        qdd2_wz = new DoubleYoVariable("qdd2_wz", registry);
        energy = new DoubleYoVariable("energy", registry);
    }
    public void doControl()
    {
        energy.set(  M1 * G * q_z.getDoubleValue()
                + 0.5 * M1 * qd_x.getDoubleValue() * qd_x.getDoubleValue()
                + 0.5 * M1 * qd_y.getDoubleValue() * qd_y.getDoubleValue()
                + 0.5 * M1 * qd_z.getDoubleValue() * qd_z.getDoubleValue()
                + 0.5 * Ixx1 * qd_wx.getDoubleValue() * qd_wx.getDoubleValue()
                + 0.5 * Iyy1 * qd_wy.getDoubleValue() * qd_wy.getDoubleValue()
                + 0.5 * Izz1 * qd_wz.getDoubleValue() * qd_wz.getDoubleValue());

        qdd2_wx.set((Iyy1 - Izz1) / Ixx1 * qd_wy.getDoubleValue() * qd_wz.getDoubleValue());
        qdd2_wy.set((Izz1 - Ixx1) / Iyy1 * qd_wz.getDoubleValue() * qd_wx.getDoubleValue());
        qdd2_wz.set((Ixx1 - Iyy1) / Izz1 * qd_wx.getDoubleValue() * qd_wy.getDoubleValue());
    }
    public YoVariableRegistry getYoVariableRegistry()
    {
        return registry;
    }

    public void initialize()
    {
    }
    public String getDescription()
    {
        return getName();
    }
}
```
</details>

<details>
<summary>WavyGroundProfile</summary>

```java
package us.ihmc.exampleSimulations.fallingBrick;
import javax.vecmath.Point3d;
import javax.vecmath.Vector3d;
import us.ihmc.jMonkeyEngineToolkit.GroundProfile3D;
import us.ihmc.jMonkeyEngineToolkit.HeightMapWithNormals;
import us.ihmc.robotics.geometry.BoundingBox3d;

public class WavyGroundProfile implements GroundProfile3D, HeightMapWithNormals
{
    private double xMin = -2.0, xMax = 2.0, yMin = -2.0, yMax = 2.0, zMin = -10.0, zMax = 10.0;

    private BoundingBox3d boundingBox = new BoundingBox3d(new Point3d(xMin, yMin, zMin), new Point3d(xMax, yMax, zMax));
    public WavyGroundProfile()
    {
    }
    public double heightAndNormalAt(double x, double y, double z, Vector3d normalToPack)
    {
        double heightAt = heightAt(x, y, z);
        surfaceNormalAt(x, y, heightAt, normalToPack);
        return heightAt;
    }

    public double heightAt(double x, double y, double z)
    {
        if ((x > xMin) && (x < xMax) && (y > yMin) && (y < yMax))
            return 1.0 * Math.exp(-Math.abs(2.0 * x)) * Math.exp(-Math.abs(2.0 * y)) * Math.sin(2.0 * Math.PI * 0.7 * x);
        else
            return 0.0;
    }
    public void surfaceNormalAt(double x, double y, double z, Vector3d normal)
    {
        normal.x = 0.0;
        normal.y = 0.0;
        normal.z = 1.0;
    }
    public void closestIntersectionTo(double x, double y, double z, Point3d point)
    {
        point.x = x;
        point.y = y;
        point.z = heightAt(x, y, z);
    }
    public void closestIntersectionAndNormalAt(double x, double y, double z, Point3d point, Vector3d normal)
    {
        closestIntersectionTo(x, y, z, point);
        surfaceNormalAt(x, y, z, normal);
    }
    public boolean checkIfInside(double x, double y, double z, Point3d intersectionToPack, Vector3d normalToPack)
    {
        closestIntersectionTo(x, y, z, intersectionToPack);
        surfaceNormalAt(x, y, z, normalToPack);

        return (z < intersectionToPack.getZ());
    }

    public boolean isClose(double x, double y, double z)
    {
        return boundingBox.isInside(x, y, z);
    }
    public BoundingBox3d getBoundingBox()
    {
        return boundingBox;
    }
    public HeightMapWithNormals getHeightMapIfAvailable()
    {
        return this;
    }
}
```
</details>
