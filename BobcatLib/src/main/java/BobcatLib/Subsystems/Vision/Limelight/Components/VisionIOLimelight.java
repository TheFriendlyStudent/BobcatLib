package BobcatLib.Subsystems.Vision.Limelight.Components;

import org.littletonrobotics.junction.Logger;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.math.numbers.N3;
import BobcatLib.Subsystems.Vision.Components.VisionIO;
import BobcatLib.Subsystems.Vision.Limelight.LimelightCamera;
import BobcatLib.Subsystems.Vision.Limelight.Utility.LimelightUtils;

public class VisionIOLimelight implements VisionIO{
    /** Creates a new VisionIOLimelight. */
      public final String name;
      public final double verticalFOV;
      public final double horizontalFOV;
      public final double limelightMountHeight;
      public final int detectorPiplineIndex; 
      public final int apriltagPipelineIndex;
      public final int horPixels;
      public final double filterTimeConstant; // in seconds, inputs occuring over a time period significantly shorter than this will be thrown out
      public final Vector<N3> visionMeasurementStdDevs;
      public final int movingAverageNumTaps;
      public final LinearFilter distanceFilter;
      public final LimelightCamera camera;
  
  
    public VisionIOLimelight(limelightConstants limelightConstants) {
      name = limelightConstants.name;
      this.camera = new LimelightCamera(name);
      verticalFOV = limelightConstants.verticalFOV;
      horizontalFOV = limelightConstants.horizontalFOV;
      limelightMountHeight=limelightConstants.limelightMountHeight;
      detectorPiplineIndex=limelightConstants.detectorPiplineIndex;
      apriltagPipelineIndex=limelightConstants.apriltagPipelineIndex;
      horPixels=limelightConstants.horPixels;
      filterTimeConstant=limelightConstants.filterTimeConstant;
      visionMeasurementStdDevs=limelightConstants.visionMeasurementStdDevs;
      movingAverageNumTaps=limelightConstants.movingAverageNumTaps;
      distanceFilter = LinearFilter.movingAverage(movingAverageNumTaps);
    }
  
    @Override
    public void updateInputs(VisionIOInputs inputs) {
      inputs.ledMode = camera.getSettings().getLedMode();
      inputs.camMode = camera.getSettings().getCamMode();
      inputs.pipelineID = camera.getData().getPipelineData().getCurrentPipelineIndex();
      inputs.pipelineLatency = camera.getData().getPipelineData().getProcessingLatency();
      inputs.ta =camera.getData().targetData.getTargetArea();
      inputs.tv =camera.getData().targetData.getTargetStatus();
      inputs.tx = camera.getData().targetData.getHorizontalOffset();
      inputs.ty = camera.getData().targetData.getVerticalOffset();
      inputs.fiducialID = camera.getData().targetData.getAprilTagID();
      inputs.boundingHorizontalPixels = LimelightUtils.getLimelightNTDouble(name, "thor");
      inputs.tClass= LimelightHelpersFast.getNeuralClassID(name);
      inputs.name=name;
    }
  
  
    public double pixlesToPercent(double pixels){
      Logger.recordOutput("Limelight/horPercent", pixels/horPixels);
      return pixels/horPixels;
    }

  }