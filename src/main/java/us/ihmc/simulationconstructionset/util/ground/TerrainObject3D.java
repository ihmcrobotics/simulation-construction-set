package us.ihmc.simulationconstructionset.util.ground;

import java.util.List;

import us.ihmc.euclid.shape.primitives.interfaces.Shape3DReadOnly;
import us.ihmc.graphicsDescription.Graphics3DObject;
import us.ihmc.jMonkeyEngineToolkit.GroundProfile3D;

public interface TerrainObject3D extends GroundProfile3D
{
   Graphics3DObject getLinkGraphics();

   default List<? extends Shape3DReadOnly> getTerrainCollisionShapes()
   {
      return null;
   }
}