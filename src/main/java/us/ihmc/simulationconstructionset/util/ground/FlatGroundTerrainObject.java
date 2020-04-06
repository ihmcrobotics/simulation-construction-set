package us.ihmc.simulationconstructionset.util.ground;

import java.util.ArrayList;
import java.util.List;

import us.ihmc.euclid.shape.primitives.Box3D;
import us.ihmc.euclid.shape.primitives.interfaces.Shape3DReadOnly;
import us.ihmc.graphicsDescription.Graphics3DObject;

public class FlatGroundTerrainObject extends FlatGroundProfile implements TerrainObject3D
{
   private static final double groundSize = 20.0;
   private static final double groundThickness = 0.03;
   
   private final Graphics3DObject groundGraphics;
   
   private final ArrayList<Shape3DReadOnly> terrainCollisionShapes = new ArrayList<>();
   
   public FlatGroundTerrainObject()
   {
      groundGraphics = new Graphics3DObject();
      groundGraphics.translate(0.0, 0.0, -groundThickness);
      groundGraphics.addCube(groundSize, groundSize, groundThickness);
      
      Box3D boxShape = new Box3D(groundSize, groundSize, groundThickness);
      boxShape.getPose().getTranslation().set(0.0, 0.0, -groundThickness);  
      this.terrainCollisionShapes.add(boxShape);
   }
   
   @Override
   public Graphics3DObject getLinkGraphics()
   {
      return groundGraphics;
   }

   @Override
   public List<Shape3DReadOnly> getTerrainCollisionShapes()
   {
      return terrainCollisionShapes;
   }
}
