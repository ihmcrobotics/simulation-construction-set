package us.ihmc.simulationconstructionset.gui.camera;

import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.jMonkeyEngineToolkit.camera.CameraTrackingAndDollyPositionHolder;
import us.ihmc.yoVariables.dataBuffer.YoVariableHolder;
import us.ihmc.yoVariables.variable.YoDouble;

public class CameraTrackAndDollyYoVariablesHolder implements CameraTrackingAndDollyPositionHolder
{
   private YoDouble track_x_var, track_y_var, track_z_var, dolly_x_var, dolly_y_var, dolly_z_var;
   private YoDouble field_of_view_var;
   
   public CameraTrackAndDollyYoVariablesHolder(YoVariableHolder holder)
   { 
      if (holder != null)
      {
         if (holder.hasUniqueVariable("q_x"))
         {
            setTrackXVar((YoDouble) holder.getVariable("q_x"));
            setDollyXVar((YoDouble) holder.getVariable("q_x"));
         }

         if (holder.hasUniqueVariable("q_y"))
         {
            setTrackYVar((YoDouble) holder.getVariable("q_y"));
            setDollyYVar((YoDouble) holder.getVariable("q_y"));
         }

         if (holder.hasUniqueVariable("q_z"))
         {
            setTrackZVar((YoDouble) holder.getVariable("q_z"));
            setDollyZVar((YoDouble) holder.getVariable("q_z"));
         }
      }

   }

   @Override
   public void getTrackingPosition(Point3D trackPositionToPack)
   {
      if (track_x_var != null)
      {
         trackPositionToPack.setX(track_x_var.getDoubleValue());
      }
      if (track_y_var != null)
      {
         trackPositionToPack.setY(track_y_var.getDoubleValue());
      }
      if (track_z_var != null)
      {
         trackPositionToPack.setZ(track_z_var.getDoubleValue());
      }
   }

   @Override
   public void getDollyPosition(Point3D dollyPositionToPack)
   {
      if (dolly_x_var != null)
      {
         dollyPositionToPack.setX(dolly_x_var.getDoubleValue());
      }
      if (dolly_y_var != null)
      {
         dollyPositionToPack.setY(dolly_y_var.getDoubleValue());
      }
      if (dolly_z_var != null)
      {
         dollyPositionToPack.setZ(dolly_z_var.getDoubleValue());
      }
      
   }
   
   
   public void setTrackingVars(YoDouble xVar, YoDouble yVar, YoDouble zVar)
   {
      if (xVar != null)
         track_x_var = xVar;
      if (yVar != null)
         track_y_var = yVar;
      if (zVar != null)
         track_z_var = zVar;
   }

   public void setDollyVars(YoDouble xVar, YoDouble yVar, YoDouble zVar)
   {
      if (xVar != null)
         dolly_x_var = xVar;
      if (yVar != null)
         dolly_y_var = yVar;
      if (zVar != null)
         dolly_z_var = zVar;
   }
   
   public void setTrackXVar(YoDouble track_x_var)
   {
      this.track_x_var = track_x_var;
   }

   public void setTrackYVar(YoDouble track_y_var)
   {
      this.track_y_var = track_y_var;
   }

   public void setTrackZVar(YoDouble track_z_var)
   {
      this.track_z_var = track_z_var;
   }

   public void setDollyXVar(YoDouble dolly_x_var)
   {
      this.dolly_x_var = dolly_x_var;
   }

   public void setDollyYVar(YoDouble dolly_y_var)
   {
      this.dolly_y_var = dolly_y_var;
   }

   public void setDollyZVar(YoDouble dolly_z_var)
   {
      this.dolly_z_var = dolly_z_var;
   }

   public void setFieldOfViewVar(YoDouble field_of_view_var)
   {
      this.field_of_view_var = field_of_view_var;
   }

   @Override
   public double getFieldOfView()
   {
      if (field_of_view_var == null)
      {
         return Double.NaN;
      }
      
      else return field_of_view_var.getDoubleValue();
   }

   
   
   @Override
   public double getTrackingX()
   {
      if (track_x_var != null)
         return track_x_var.getDoubleValue();
      else
         return 0.0;
   }

   @Override
   public double getTrackingY()
   {
      if (track_y_var != null)
         return track_y_var.getDoubleValue();
      else
         return 0.0;
   }

   @Override
   public double getTrackingZ()
   {
      if (track_z_var != null)
         return track_z_var.getDoubleValue();
      else
         return 0.0;
   }

   @Override
   public double getDollyX()
   {
      if (dolly_x_var != null)
         return dolly_x_var.getDoubleValue();
      else
         return 0.0;
   }

   @Override
   public double getDollyY()
   {
      if (dolly_y_var != null)
         return dolly_y_var.getDoubleValue();
      else
         return 0.0;
   }

   @Override
   public double getDollyZ()
   {
      if (dolly_z_var != null)
         return dolly_z_var.getDoubleValue();
      else
         return 0.0;
   }

   @Override
   public void closeAndDispose()
   {
      track_x_var = track_y_var = track_z_var = dolly_x_var = dolly_y_var = dolly_z_var = null;
      field_of_view_var = null;
   }

}
