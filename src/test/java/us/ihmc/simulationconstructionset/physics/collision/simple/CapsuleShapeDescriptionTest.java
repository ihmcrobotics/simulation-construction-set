package us.ihmc.simulationconstructionset.physics.collision.simple;

import org.junit.jupiter.api.Test;

import us.ihmc.euclid.geometry.BoundingBox3D;
import us.ihmc.euclid.tools.EuclidCoreTestTools;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Point3D;

public class CapsuleShapeDescriptionTest
{
   @Test // timeout = 30000
   public void test()
   {
      double radius = 0.2;
      double height = 0.6;

      CapsuleShapeDescription<?> capsule = new CapsuleShapeDescription<>(radius, height);

      BoundingBox3D boundingBox = new BoundingBox3D(0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
      capsule.getBoundingBox(boundingBox);

      EuclidCoreTestTools.assertEquals(new Point3D(-0.2, -0.2, -0.3), boundingBox.getMinPoint(), 1e-10);
      EuclidCoreTestTools.assertEquals(new Point3D(0.2, 0.2, 0.3), boundingBox.getMaxPoint(), 1e-10);

      RigidBodyTransform transform = new RigidBodyTransform();
      transform.getTranslation().set(7.0, 8.0, 9.0);
      capsule.applyTransform(transform);

      capsule.getBoundingBox(boundingBox);

      EuclidCoreTestTools.assertEquals(new Point3D(6.8, 7.8, 8.7), boundingBox.getMinPoint(), 1e-10);
      EuclidCoreTestTools.assertEquals(new Point3D(7.2, 8.2, 9.3), boundingBox.getMaxPoint(), 1e-10);
   }

}
