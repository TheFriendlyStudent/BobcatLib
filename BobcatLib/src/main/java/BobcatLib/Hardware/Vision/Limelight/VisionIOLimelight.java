// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package BobcatLib.Hardware.Vision.Limelight;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.NetworkTableInstance;
import BobcatLib.Hardware.Vision.Limelight.LimelightHelpers;
import BobcatLib.Utilities.BobcatUtil;

public class VisionIOLimelight implements VisionIO{
  /** Creates a new VisionIOLimelight. */
    LEDMode currentLedMode = LEDMode.FORCEOFF;
    CamMode currentCamMode = CamMode.VISION;
    public final limelightConstants constants;
    private final String name;

  public VisionIOLimelight(limelightConstants limelightConstants) {
    constants = limelightConstants;
    name = constants.name;
    
  }

  @Override
  public void updateInputs(VisionIOInputs inputs) {
    inputs.ledMode = currentLedMode;
    // inputs.camMode = currentCamMode;
    inputs.pipelineID = LimelightHelpers.getCurrentPipelineIndex(name);
    inputs.pipelineLatency = LimelightHelpers.getLatency_Pipeline(name);
    inputs.ta = LimelightHelpers.getTA(name);
    inputs.tv = LimelightHelpers.getTV(name);
    inputs.tx = LimelightHelpers.getTX(name);
    inputs.ty = LimelightHelpers.getTY(name);
    inputs.fiducialID = LimelightHelpers.getFiducialID(name);
    inputs.tClass=Double.parseDouble(LimelightHelpers.getNeuralClassID(name));
    inputs.name=name;
    inputs.botPoseMG2 = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(name).pose;
    inputs.tagCount = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(name).tagCount;
    inputs.avgTagDist = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(name).avgTagDist;
    inputs.botPose3d = LimelightHelpers.getBotPose3d_wpiBlue(name);
    inputs.timestamp = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(name).timestampSeconds;

  }



  @Override
  public void setLEDS(LEDMode mode) {
    switch (mode) {
      case FORCEBLINK:
        LimelightHelpers.setLEDMode_ForceBlink(name);
        currentLedMode = LEDMode.FORCEBLINK;
        break;
      case FORCEOFF:
        LimelightHelpers.setLEDMode_ForceOff(name);
        currentLedMode = LEDMode.FORCEOFF;
      case FORCEON:
        LimelightHelpers.setLEDMode_ForceOn(name);
        currentLedMode = LEDMode.FORCEON;
      case PIPELINECONTROL:
        LimelightHelpers.setLEDMode_PipelineControl(name);
        currentLedMode = LEDMode.PIPELINECONTROL;
      default:
        LimelightHelpers.setLEDMode_ForceOff(name);
        currentLedMode = LEDMode.FORCEOFF;
        break;
    }
  }

  // @Override
  // public void setCamMode(CamMode mode){
  //   switch (mode){
  //     case DRIVERCAM:
  //     LimelightHelpers.setCameraMode_Driver(name);
  //     currentCamMode = CamMode.DRIVERCAM;
  //     case VISION:
  //     LimelightHelpers.setCameraMode_Processor(name);
  //     currentCamMode = CamMode.VISION;
  //   }
  // }

  @Override
  public void setPipeline(String limelight, int index){    
    LimelightHelpers.setPipelineIndex(limelight, index);
  }

  @Override
  public void setRobotOrientationMG2(Rotation2d gyro){
    gyro = BobcatUtil.isBlue()? gyro : gyro.rotateBy(Rotation2d.fromDegrees(180));
    double gyroval = BobcatUtil.wrapRot2d(gyro).getDegrees();
    
    LimelightHelpers.SetRobotOrientation(name, gyroval, 0, 0, 0, 0, 0);
  }

  @Override
  public void setPermittedTags(int[] tags){
    LimelightHelpers.SetFiducialIDFiltersOverride(name, tags);
  }

  @Override
  public void setPriorityID(int tagID){
    NetworkTableInstance.getDefault().getTable(name).getEntry("priorityid").setDouble(tagID);
  }
}
