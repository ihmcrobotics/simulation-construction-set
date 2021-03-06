package us.ihmc.simulationconstructionset.util.ground;

import java.util.ArrayList;

import us.ihmc.euclid.geometry.BoundingBox3D;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple3D.interfaces.Point3DBasics;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DBasics;
import us.ihmc.jMonkeyEngineToolkit.CombinedHeightMapWithNormals;
import us.ihmc.jMonkeyEngineToolkit.GroundProfile3D;
import us.ihmc.jMonkeyEngineToolkit.HeightMapWithNormals;

public class CombinedGroundProfile3D implements GroundProfile3D
{
   private GroundProfile3D[] groundProfiles;
   private final BoundingBox3D boundingBox;
   private final CombinedHeightMapWithNormals heightMap;

   public CombinedGroundProfile3D(GroundProfile3D[] groundProfiles)
   {
      this.groundProfiles = groundProfiles;
      heightMap = new CombinedHeightMapWithNormals();

      BoundingBox3D boundingBox = null;
      for (GroundProfile3D groundProfile : groundProfiles)
      {
         if (boundingBox == null)
            boundingBox = groundProfile.getBoundingBox();
         else
         {
            boundingBox = BoundingBox3D.union(boundingBox, groundProfile.getBoundingBox());
         }

         HeightMapWithNormals heightMapIfAvailable = groundProfile.getHeightMapIfAvailable();
         if (heightMapIfAvailable != null)
            heightMap.addHeightMap(heightMapIfAvailable);
      }

      this.boundingBox = boundingBox;
   }

   public CombinedGroundProfile3D(ArrayList<GroundProfile3D> groundProfilesArrayList)
   {
      groundProfiles = new GroundProfile3D[groundProfilesArrayList.size()];
      heightMap = new CombinedHeightMapWithNormals();

      BoundingBox3D boundingBox = null;
      for (GroundProfile3D groundProfile : groundProfiles)
      {
         if (boundingBox == null)
            boundingBox = groundProfile.getBoundingBox();
         else
         {
            boundingBox = BoundingBox3D.union(boundingBox, groundProfile.getBoundingBox());
         }

         HeightMapWithNormals heightMapIfAvailable = groundProfile.getHeightMapIfAvailable();
         if (heightMapIfAvailable != null)
            heightMap.addHeightMap(heightMapIfAvailable);
      }
      this.boundingBox = boundingBox;

      groundProfilesArrayList.toArray(groundProfiles);
   }

   public GroundProfile3D[] getGroundProfiles()
   {
      return groundProfiles;
   }

   private final Point3D tempPointToCheck = new Point3D();

   @Override
   public boolean checkIfInside(double x, double y, double z, Point3DBasics intersectionToPack, Vector3DBasics normalToPack)
   {
      double smallestDistance = Double.MAX_VALUE;
      Point3D localIntersection = new Point3D();
      Vector3D localNormal = new Vector3D();
      boolean isInside = false;

      tempPointToCheck.set(x, y, z);

      // Pre-set some values, in case no object is close.
      intersectionToPack.set(x, y, 0.0);
      normalToPack.set(0.0, 0.0, 1.0);

      for (GroundProfile3D groundProfile : groundProfiles)
      {
         if (groundProfile.isClose(x, y, z))
         {
            boolean localIsInside = groundProfile.checkIfInside(x, y, z, localIntersection, localNormal);

            if (localIsInside && (tempPointToCheck.distance(localIntersection) < smallestDistance))
            {
               smallestDistance = tempPointToCheck.distance(localIntersection);
               intersectionToPack.set(localIntersection);
               normalToPack.set(localNormal);
               isInside = true;
            }
         }
      }

      // Reset pointToCheck for rewindability tests
      tempPointToCheck.set(0.0, 0.0, 0.0);

      return isInside;
   }

   @Override
   public boolean isClose(double x, double y, double z)
   {
      if (boundingBox == null)
         return false;

      return boundingBox.isInsideInclusive(x, y, z);
   }

   @Override
   public BoundingBox3D getBoundingBox()
   {
      return boundingBox;
   }

   @Override
   public HeightMapWithNormals getHeightMapIfAvailable()
   {
      return heightMap;
   }
}
