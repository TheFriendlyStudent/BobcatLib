package BobcatLib.Subsystems.Vision.Components;

import org.littletonrobotics.junction.AutoLog;

import BobcatLib.Subsystems.Vision.Limelight.Utility.LimelightSettings.CamMode;
import BobcatLib.Subsystems.Vision.Limelight.Utility.LimelightSettings.LEDMode;



/** Vision subsystem hardware interface. */
public interface VisionIO {
  /** The set of loggable inputs for the vision subsystem. */
  
  @AutoLog
  public static class VisionIOInputs{  
      public LEDMode ledMode = LEDMode.ForceOff;
      public double pipelineID = 0;
      public double pipelineLatency = 0;
      public double ta;
      public boolean tv;
      public double tx;
      public double ty;
      public double fiducialID;
      public double boundingHorizontalPixels;
      public double tClass;
      public String name;
      public CamMode camMode = CamMode.Vision;
    }
      /** Updates the set of loggable inputs. */
    public default void updateInputs(VisionIOInputs inputs) {}

      /** Sets the pipeline number. */
    public default void setLEDS(LEDMode mode) {}

    public default void setPipeline(String limelight, int index){}

    public default double pixlesToPercent(double pixels){
      return 0.0;
    }
    public default double getTClass(){
      return 0.0;
    }

    public default void setCamMode(CamMode mode){}

  /**
   * 
   * @param widthPercent [0,1], percentage of the vertical width of the image that the note is taking up
   * @return distance in meters
   */
  public default double distanceFromCameraPercentage(double widthPercent){
   return 0.0;
  }
}