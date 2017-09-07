package us.ihmc.simulationconstructionset;

import java.util.ArrayList;

import us.ihmc.simulationconstructionset.physics.CollisionArbiter;
import us.ihmc.simulationconstructionset.physics.CollisionHandler;
import us.ihmc.simulationconstructionset.physics.ScsCollisionDetector;
import us.ihmc.simulationconstructionset.physics.collision.CollisionDetectionResult;
import us.ihmc.simulationconstructionset.physics.collision.DefaultCollisionVisualizer;
import us.ihmc.simulationconstructionset.scripts.Script;
import us.ihmc.simulationconstructionset.synchronization.SimulationSynchronizer;

public class Simulator implements java.io.Serializable
{
   private static final long serialVersionUID = -3492879446149849938L;

   private final SimulationSynchronizer simulationSynchronizer;
   private Robot[] robots;

   private double DT;
   private ArrayList<Script> scripts = null;

   private ScsCollisionDetector collisionDetector;
   private CollisionArbiter collisionArbiter;
   private CollisionHandler collisionHandler;
   private DefaultCollisionVisualizer collisionVisualizer;

   // private final YoVariable time;

   public Simulator(SimulationSynchronizer simulationSynchronizer, Robot[] robots, double dt)
   {
      this.simulationSynchronizer = simulationSynchronizer;
      this.robots = robots;
      this.DT = dt;

      //    this.time = time;
   }

   public void setRobots(Robot[] robots)
   {
      this.robots = robots;
   }

   public void setDT(double simulateDT)
   {
      DT = simulateDT;
   }

   public double getDT()
   {
      return this.DT;
   }

   protected void addScript(Script script)
   {
      if (scripts == null)
         scripts = new ArrayList<Script>();
      scripts.add(script);
   }

   protected void simulate() throws UnreasonableAccelerationException
   {
      updateState();
      doControl();
      doDynamicsAndIntegrate();
   }

   private final CollisionDetectionResult newCollisions = new CollisionDetectionResult();

   private void updateState()
   {
      synchronized (simulationSynchronizer)
      {
         for (Robot robot : robots)
         {
            // +++JEP090122 This updates the locations and velocities of everything. Need to do this instead of just update so that the ground contact has the
            // most recent velocities and is rewindable...
            robot.updateVelocities();

            //          rob.update();

            robot.updateAllGroundContactPointVelocities(); // +++JEP OPTIMIZE: Need to do this once so all point velocities get updated. Otherwise only those in contact will.


            if (robot.getGroundContactModel() != null)
            {
               robot.getGroundContactModel().doGroundContact(); // Do the ground contact model
            }

            // Needed to move this outside and do it even if no ground contact model, for
            // Contact models that are done outside of the robot.
            robot.decideGroundContactPointsInContact(); // +++JEP OPTIMIZE. This should be in a GroundContactDetector...

            if (scripts != null) // Run the scripts
            {
               for (Script script : scripts)
               {
                  script.doScript(robot.getTime());
               }
            }
         }

         if (collisionDetector != null)
         {
            if (collisionVisualizer != null)
               collisionVisualizer.callBeforeCollisionDetection();

            newCollisions.clear();
            collisionDetector.performCollisionDetection(newCollisions);
            collisionArbiter.processNewCollisions(newCollisions);
            CollisionDetectionResult cachedCollisions = collisionArbiter.getCollisions();

            collisionHandler.handleCollisions(cachedCollisions);
         }
      }
   }

   protected void doControl()
   {
      for (Robot robot : robots)
      {
         // +++JEP 7/18/2005. Needed to take doControl out of the synchronized block in case the controller is trying
         // to render an image for control. I don't think this will affect things or make glitches, but I'm not absolutely sure...

         // if (rob.getController() != null) rob.getController().doControl(); // Do the controller
         robot.doControllers();
      }
   }

   protected void doDynamicsAndIntegrate() throws UnreasonableAccelerationException
   {
      synchronized (simulationSynchronizer)
      {
         for (int i = 0; i < robots.length; i++)
         {
            Robot robot = robots[i];
            robot.doDynamicsAndIntegrate(DT);
            robot.updateIMUMountAccelerations();
         }
      }
   }

   protected void forceClassLoading()
   {
      //
   }

   public void setCollisions(ScsCollisionDetector collisionDetector, CollisionArbiter collisionArbiter, CollisionHandler collisionHandler, DefaultCollisionVisualizer visulize)
   {
      this.collisionDetector = collisionDetector;
      this.collisionArbiter = collisionArbiter;
      this.collisionHandler = collisionHandler;
      this.collisionVisualizer = visulize;
   }
}
