package us.ihmc.simulationconstructionset.physics.collision.simple;

import static us.ihmc.robotics.Assert.assertEquals;

import org.junit.jupiter.api.Test;

import us.ihmc.commons.MutationTestFacilitator;
import us.ihmc.euclid.geometry.BoundingBox3D;
import us.ihmc.euclid.tools.EuclidCoreTestTools;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Point3D;

public class CylinderShapeDescriptionTest
{

   @Test // timeout = 30000
   public void testProjectionWhenNotTransformed()
   {
      double radius = 0.5;
      double height = 0.1;
      CylinderShapeDescription<?> cylinder = new CylinderShapeDescription<>(radius, height);
      assertEquals(radius, cylinder.getRadius(), 1e-10);
      assertEquals(height, cylinder.getHeight(), 1e-10);
      assertEquals(0.0, cylinder.getSmoothingRadius(), 1e-10);

      Point3D pointToProject = new Point3D(0.0, 0.0, 100.0);
      Point3D closestPointOnCylinder = new Point3D();
      cylinder.getProjection(pointToProject, closestPointOnCylinder);
      EuclidCoreTestTools.assertEquals(new Point3D(0.0, 0.0, height / 2.0), closestPointOnCylinder, 1e-7);

      pointToProject = new Point3D(0.0, 0.0, -100.0);
      cylinder.getProjection(pointToProject, closestPointOnCylinder);
      EuclidCoreTestTools.assertEquals(new Point3D(0.0, 0.0, -height / 2.0), closestPointOnCylinder, 1e-7);

      pointToProject = new Point3D(100.0, 0.0, 10.0);
      cylinder.getProjection(pointToProject, closestPointOnCylinder);
      EuclidCoreTestTools.assertEquals(new Point3D(radius, 0.0, height / 2.0), closestPointOnCylinder, 1e-7);

      pointToProject = new Point3D(0.0, -20.0, 10.0);
      cylinder.getProjection(pointToProject, closestPointOnCylinder);
      EuclidCoreTestTools.assertEquals(new Point3D(0.0, -radius, height / 2.0), closestPointOnCylinder, 1e-7);

      pointToProject = new Point3D(10.0, 10.0, height * 0.1);
      cylinder.getProjection(pointToProject, closestPointOnCylinder);
      EuclidCoreTestTools.assertEquals(new Point3D(radius * Math.sqrt(2.0) / 2.0, radius * Math.sqrt(2.0) / 2.0, height * 0.1),
                                              closestPointOnCylinder,
                                              1e-7);
   }

   @Test // timeout = 30000
   public void testProjectionWhenRotated()
   {
      double radius = 0.5;
      double height = 0.1;
      CylinderShapeDescription<?> cylinder = new CylinderShapeDescription<>(radius, height);
      RigidBodyTransform transform = new RigidBodyTransform();

      double angle = Math.PI / 7.0;
      transform.appendPitchRotation(angle);
      cylinder.applyTransform(transform);

      Point3D expectedPoint = new Point3D(-radius, 0.0, height / 2.0);
      transform.transform(expectedPoint);

      Point3D pointToProject = new Point3D(0.0, 0.0, 100.0);
      Point3D closestPointOnCylinder = new Point3D();
      cylinder.getProjection(pointToProject, closestPointOnCylinder);
      EuclidCoreTestTools.assertEquals(expectedPoint, closestPointOnCylinder, 1e-7);

      pointToProject = new Point3D(0.0, 0.0, -100.0);
      expectedPoint = new Point3D(radius, 0.0, -height / 2.0);
      transform.transform(expectedPoint);
      cylinder.getProjection(pointToProject, closestPointOnCylinder);
      EuclidCoreTestTools.assertEquals(expectedPoint, closestPointOnCylinder, 1e-7);

      pointToProject = new Point3D(100.0, 0.0, 10.0);
      expectedPoint = new Point3D(radius, 0.0, height / 2.0);
      transform.transform(expectedPoint);
      cylinder.getProjection(pointToProject, closestPointOnCylinder);
      EuclidCoreTestTools.assertEquals(expectedPoint, closestPointOnCylinder, 1e-7);
   }

   @Test // timeout = 30000
   public void testBoundingBox()
   {
      double radius = 0.5;
      double height = 0.1;
      CylinderShapeDescription<?> cylinder = new CylinderShapeDescription<>(radius, height);
      BoundingBox3D boundingBox = new BoundingBox3D(0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
      cylinder.getBoundingBox(boundingBox);

      EuclidCoreTestTools.assertEquals(new Point3D(-0.5, -0.5, -0.05), boundingBox.getMinPoint(), 1e-10);
      EuclidCoreTestTools.assertEquals(new Point3D(0.5, 0.5, 0.05), boundingBox.getMaxPoint(), 1e-10);

      RigidBodyTransform transform = new RigidBodyTransform();
      transform.setRotationEulerAndZeroTranslation(Math.PI / 4.0, 0.0, 0.0);
      cylinder.applyTransform(transform);

      cylinder.getBoundingBox(boundingBox);

      EuclidCoreTestTools.assertEquals(new Point3D(-0.5, -0.38890872965260115, -0.3889087296526011), boundingBox.getMinPoint(), 1e-10);
      EuclidCoreTestTools.assertEquals(new Point3D(0.5, 0.38890872965260115, 0.3889087296526011), boundingBox.getMaxPoint(), 1e-10);
   }

   public static void main(String[] args)
   {
      MutationTestFacilitator.facilitateMutationTestForClass(CylinderShapeDescription.class, CylinderShapeDescriptionTest.class);
   }
}
