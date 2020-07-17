package us.ihmc.simulationconstructionset;

import java.io.Serializable;
import java.util.ArrayList;
import java.util.List;

import us.ihmc.graphicsDescription.HeightMap;
import us.ihmc.jMonkeyEngineToolkit.Graphics3DAdapter;
import us.ihmc.simulationconstructionset.graphics.GraphicsRobot;
import us.ihmc.simulationconstructionset.physics.CollisionArbiter;
import us.ihmc.simulationconstructionset.physics.ScsPhysics;
import us.ihmc.simulationconstructionset.physics.collision.simple.CollisionManager;
import us.ihmc.simulationconstructionset.physics.collision.simple.DoNothingCollisionArbiter;
import us.ihmc.simulationconstructionset.scripts.Script;
import us.ihmc.simulationconstructionset.synchronization.SimulationSynchronizer;
import us.ihmc.simulationconstructionset.util.ground.FlatGroundProfile;
import us.ihmc.yoVariables.dataBuffer.DataBuffer;
import us.ihmc.yoVariables.registry.NameSpace;
import us.ihmc.yoVariables.registry.YoVariableHolder;
import us.ihmc.yoVariables.registry.YoVariableList;
import us.ihmc.yoVariables.variable.YoVariable;

public class Simulation implements YoVariableHolder, Serializable // Runnable,
{
   private static final long serialVersionUID = 8438645717978048239L;

   private CollisionManager collisionManager;

   private Graphics3DAdapter myGraphics;

   private final SimulationSynchronizer simulationSynchronizer;

   // //////////////////////////////////////////////////
   // private double DEFAULT_DT = 0.0004;
   // private int DEFAULT_RECORD_FREQ = 50;

   private double SIMULATION_DT = 0.0004;
   private int RECORD_FREQ = 1;

   private Robot[] robots;

   // private final YoVariable time;
   private Simulator mySimulator;
   private DataBuffer myDataBuffer;

   private YoVariableList myCombinedVarList = new YoVariableList("Combined");

   private List<SimulationDoneListener> simulateDoneListeners = new ArrayList<>();
   private List<SimulationDoneCriterion> simulateDoneCriterions;

   // private VarList robVarList, gcVarList; //controllerVarList,
   // private List<VarList> controllerVarLists = new ArrayList<VarList>();

   public void initPhysics(ScsPhysics physics)
   {
      mySimulator.setCollisions(physics.collisionDetector, physics.collisionArbiter, physics.collisionHandler, physics.visualize);

      for (Robot robot : robots)
      {
         if (physics.collisionConfigure != null)
            physics.collisionConfigure.setup(robot, physics.collisionDetector, physics.collisionHandler);
      }
   }

   public double getDT()
   {
      if (mySimulator != null)
         return mySimulator.getDT();
      else
         return 1.0;
   }

   public void setDT(double simulateDT, int recordFrequency)
   {
      if (mySimulator != null)
         mySimulator.setDT(simulateDT);

      SIMULATION_DT = simulateDT;
      RECORD_FREQ = recordFrequency;

      // recomputeTiming();
   }

   public void setRecordDT(double recordDT)
   {
      RECORD_FREQ = (int) Math.round(recordDT / mySimulator.getDT());
      if (RECORD_FREQ < 1)
         RECORD_FREQ = 1;

      // recomputeTiming();
   }

   public long getRecordFreq()
   {
      return RECORD_FREQ;
   }

   public void addScript(Script script)
   {
      mySimulator.addScript(script);
   }

   @Override
   public List<YoVariable> getVariables()
   {
      return myDataBuffer.getVariables();
   }

   @Override
   public YoVariable findVariable(String varname)
   {
      return myDataBuffer.getYoVariableList().findVariable(varname);
   }

   @Override
   public boolean hasUniqueVariable(String varname)
   {
      return myDataBuffer.getYoVariableList().hasUniqueVariable(varname);
   }

   @Override
   public YoVariable findVariable(String nameSpace, String varname)
   {
      return myDataBuffer.getYoVariableList().findVariable(nameSpace, varname);
   }

   @Override
   public boolean hasUniqueVariable(String nameSpace, String varname)
   {
      return myDataBuffer.getYoVariableList().hasUniqueVariable(nameSpace, varname);
   }

   @Override
   public List<YoVariable> findVariables(String nameSpace, String varname)
   {
      return myDataBuffer.getYoVariableList().findVariables(nameSpace, varname);
   }

   @Override
   public List<YoVariable> findVariables(String varname)
   {
      return myDataBuffer.getYoVariableList().findVariables(varname);
   }

   @Override
   public List<YoVariable> findVariables(NameSpace nameSpace)
   {
      return myDataBuffer.getYoVariableList().findVariables(nameSpace);
   }

   public void registerVariable(YoVariable variable)
   {
      throw new RuntimeException("Do not register variables with a Simulation.java!");
   }

   public List<YoVariable> getVariablesThatContain(String searchString, boolean caseSensitive)
   {
      return myDataBuffer.getVariablesThatContain(searchString, caseSensitive, getVariables());
   }

   public List<YoVariable> getVariablesThatStartWith(String searchString)
   {
      return myDataBuffer.getVariablesThatStartWith(searchString);
   }

   public List<YoVariable> getVars(String[] varNames, String[] regularExpressions)
   {
      return myDataBuffer.getVars(varNames, regularExpressions);
   }

   // public Simulation(int dataBufferSize)
   // {
   //   this(((Robot) null), dataBufferSize);
   // }

   public Simulation(Robot robot, int dataBufferSize)
   {
      this(new Robot[] {robot}, dataBufferSize);
   }

   public Simulation(Robot[] robots, int dataBufferSize)
   {
      simulationSynchronizer = new SimulationSynchronizer();

      // Make sure robots actually has some robots in it
      if ((robots != null) && (robots[0] == null))
         robots = null;

      // Create a data buffer:
      myDataBuffer = new DataBuffer(dataBufferSize);
      myDataBuffer.setWrapBuffer(true);

      setRobots(robots);
   }

   public void closeAndDispose()
   {
      myDataBuffer.closeAndDispose();

      myDataBuffer = null;
      mySimulator = null;
   }

   public void setRobots(Robot[] robots)
   {
      this.robots = robots;
      mySimulator = new Simulator(simulationSynchronizer, robots, SIMULATION_DT);
      setDT(SIMULATION_DT, RECORD_FREQ);

      if (robots != null)
      {
         for (Robot robot : robots)
         {
            addVariablesFromARobot(robot);
         }
      }

      myDataBuffer.copyValuesThrough();
      updateRobots(robots);
   }

   public void addRobot(Robot robot)
   {
      Robot[] newRobots;
      if (robots == null)
      {
         newRobots = new Robot[] {robot};
      }
      else
      {
         newRobots = new Robot[robots.length + 1];
         for (int i = 0; i < robots.length; i++)
         {
            newRobots[i] = robots[i];
         }

         newRobots[newRobots.length - 1] = robot;
      }

      robots = newRobots;

      if (mySimulator == null)
      {
         mySimulator = new Simulator(simulationSynchronizer, robots, SIMULATION_DT);
      }
      else
      {
         mySimulator.setRobots(robots);
      }

      setDT(SIMULATION_DT, RECORD_FREQ);
      addVariablesFromARobot(robot);

      myDataBuffer.copyValuesThrough();
      updateRobots(robots);
   }

   private void updateRobots(Robot[] robots)
   {
      if (robots != null)
      {
         for (Robot robot : robots)
         {
            robot.update();
         }
      }
   }

   private void addVariablesFromARobot(Robot robot)
   {
      myDataBuffer.addVariables(robot.getRobotsYoRegistry().subtreeVariables());
      myCombinedVarList.addVariables(robot.getRobotsYoRegistry().subtreeVariables());
   }

   public DataBuffer getDataBuffer()
   {
      return myDataBuffer;
   }

   public Robot[] getRobots()
   {
      return robots;
   }

   public YoVariableList getCombinedVarList()
   {
      return myCombinedVarList;
   }

   public synchronized void setSimulateDoneCriterion(SimulationDoneCriterion criterion)
   {
      //@TODO: Rename this add, not set.
      if (criterion == null)
         return;

      if (simulateDoneCriterions == null)
         simulateDoneCriterions = new ArrayList<>();
      simulateDoneCriterions.add(criterion);
   }

   public synchronized void addSimulateDoneListener(SimulationDoneListener listener)
   {
      simulateDoneListeners.add(listener);
   }

   public synchronized void removeSimulateDoneListener(SimulationDoneListener listener)
   {
      simulateDoneListeners.remove(listener);
   }

   public void notifySimulateDoneListeners()
   {
      for (int i = 0; i < simulateDoneListeners.size(); i++)
      {
         simulateDoneListeners.get(i).simulationDone();
      }
   }

   public void notifySimulateDoneListenersOfException(Throwable throwable)
   {
      for (int i = 0; i < simulateDoneListeners.size(); i++)
      {
         simulateDoneListeners.get(i).simulationDoneWithException(throwable);
      }
   }

   public boolean checkSimulateDoneCriterion()
   {
      if (simulateDoneCriterions == null)
         return false;

      for (int i = 0; i < simulateDoneCriterions.size(); i++)
      {
         if (simulateDoneCriterions.get(i).isSimulationDone())
            return true;
      }

      return false;
   }

   protected void simulate() throws UnreasonableAccelerationException
   {
      mySimulator.simulate();
   }

   public synchronized void simulate(int numTicks) throws UnreasonableAccelerationException
   {
      while (numTicks > 0)
      {
         for (int i = 0; i < RECORD_FREQ; i++)
         {
            mySimulator.simulate();
            {
               if (checkSimulateDoneCriterion())
               {
                  numTicks = -1;
               }
            }
         }

         myDataBuffer.tickAndUpdate();
         numTicks -= RECORD_FREQ;
      }

      // Notify all the listeners that the simulation stopped...
      for (int i = 0; i < simulateDoneListeners.size(); i++)
      {
         (simulateDoneListeners.get(i)).simulationDone();
      }
   }

   protected void doControl()
   {
      mySimulator.doControl();
   }

   protected void doDynamicsAndIntegrate() throws UnreasonableAccelerationException
   {
      mySimulator.doDynamicsAndIntegrate();
   }

   protected void forceClassLoading()
   {
      mySimulator.forceClassLoading();
   }

   public void tickAndUpdate()
   {
      myDataBuffer.tickAndUpdate();
   }

   public synchronized void simulate(double simulationTime) throws UnreasonableAccelerationException
   {
      simulate((int) (simulationTime / mySimulator.getDT()));
   }

   public void setupSimulationGraphics(List<GraphicsRobot> graphicsRobotsToUpdate)
   {
      // 3D Canvass Stuff goes here...
      // myGraphics = new StandardSimulationGraphics(this.rob, this.myCombinedVarList, null);
      if (robots.length > 0)
      {
         GroundContactModel groundContactModel = robots[0].getGroundContactModel();
         //         GroundProfile groundProfile = null;
         HeightMap heightMap = null;

         if (groundContactModel != null)
         {
            heightMap = HeightMapFromGroundContactModel.getHeightMap(groundContactModel);
         }
         if (heightMap == null)
            heightMap = new FlatGroundProfile();

         myGraphics = SupportedGraphics3DAdapter.instantiateDefaultGraphicsAdapter(true);

         //         HeightMapFromGroundProfile heightMap = new HeightMapFromGroundProfile(groundProfile);
         myGraphics.setHeightMap(heightMap);
         for (GraphicsRobot graphicsRobot : graphicsRobotsToUpdate)
         {
            myGraphics.addRootNode(graphicsRobot.getRootNode());
         }

      }

      // End of 3D Stuff...
   }

   public Graphics3DAdapter getSimulationGraphics()
   {
      return myGraphics;
   }

   public void addVarList(YoVariableList newVarList)
   {
      myCombinedVarList.addVariables(newVarList);
      myDataBuffer.addVariables(newVarList.getVariables());
   }

   public SimulationSynchronizer getSimulationSynchronizer()
   {
      return simulationSynchronizer;
   }

   public void initializeShapeCollision(CollisionManager collisionManager)
   {
      collisionManager.createCollisionShapesFromRobots(robots);
      collisionManager.setUpEnvironment();

      CollisionArbiter collisionArbiter = new DoNothingCollisionArbiter();
      initPhysics(new ScsPhysics(null,
                                 collisionManager.getCollisionDetector(),
                                 collisionArbiter,
                                 collisionManager.getCollisionHandler(),
                                 collisionManager.getCollisionVisualizer()));
   }
}
