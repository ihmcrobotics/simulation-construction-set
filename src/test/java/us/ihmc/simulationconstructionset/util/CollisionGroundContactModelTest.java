package us.ihmc.simulationconstructionset.util;

import us.ihmc.robotics.Assert;
import org.junit.jupiter.api.Test;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.simulationconstructionset.GroundContactPoint;
import us.ihmc.simulationconstructionset.util.ground.RollingGroundProfile;
import us.ihmc.yoVariables.registry.YoVariableRegistry;

import java.util.ArrayList;

public class CollisionGroundContactModelTest
{
   @Test// timeout = 30000
   public void testDoGroundContact()
   {
      YoVariableRegistry registry = new YoVariableRegistry("CollisionGroundContactModelTest");

      ArrayList<GroundContactPoint> gcPoints = new ArrayList<>();

      GroundContactPoint gc = new GroundContactPoint("groundContactPoint", registry);
      gc.setPosition(new Point3D(0.852, 0.116, 0.099));
      gcPoints.add(gc);

      CollisionGroundContactModel groundContactModel = new CollisionGroundContactModel(gcPoints, registry);
      groundContactModel.setGroundProfile3D(new RollingGroundProfile());

      groundContactModel.doGroundContact();

      Assert.assertTrue(gc.isInContact());

      gc.setPosition(new Point3D(0.852, 0.116, 0.15));

      groundContactModel.doGroundContact();

      Assert.assertFalse(gc.isInContact());
   }
}
