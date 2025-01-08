// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package BobcatLib.Hardware.Vision.Limelight;

import BobcatLib.Hardware.Vision.AprilTagVisionConstants;
import BobcatLib.Hardware.Vision.AprilTagVisionFieldConstants;
import BobcatLib.Hardware.Vision.Limelight.VisionIO;
import BobcatLib.Subsystems.Swerve.SimpleSwerve.SwerveDrive;
import BobcatLib.Subsystems.Swerve.SimpleSwerve.Constants.FieldConstants;
import BobcatLib.Utilities.Vision.VisionObservation;

import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
// import frc.robot.Constants.FieldConstants;
// import frc.robot.Constants.VisionConstants;

public class Vision extends SubsystemBase {
  /** Creates a new Vision. */
  private final VisionIO io;
  private final VisionIOInputsAutoLogged inputs = new VisionIOInputsAutoLogged();
  private Supplier<Rotation2d> yaw;
  private SwerveDrive swerve;

  public boolean apriltagPipeline;
  private double xyStdDev;
  private double thetaStdDev;

  public Vision(SwerveDrive swerve, VisionIO io, Supplier<Rotation2d> yaw) {
    this.io = io;
    this.yaw = yaw;
    this.swerve = swerve;

    io.setLEDS(LEDMode.FORCEOFF);
  }

  public void setLEDS(boolean on) {
    io.setLEDS(on ? LEDMode.FORCEBLINK : LEDMode.PIPELINECONTROL);
  }

  // public void setCamMode(CamMode mode) {
  //   io.setCamMode(mode);
  // }

  public double getTClass() {
    return inputs.tClass;
  }

  public boolean getTV() {
    return inputs.tv;
  }

  public double getID(){
    return inputs.fiducialID;
  }

  public void setPipeline(int id) {
    io.setPipeline(inputs.name, id);
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Limelight" + inputs.name, inputs);

    apriltagPipeline = inputs.pipelineID == 0;

    SetRobotOrientation(yaw.get());

    if(inputs.tagCount>=2){
      xyStdDev=AprilTagVisionConstants.limelightConstants.xySingleTagStdDev;
      thetaStdDev=AprilTagVisionConstants.limelightConstants.thetaSingleTagStdDev;
    }else{
      xyStdDev=AprilTagVisionConstants.limelightConstants.xyMultiTagStdDev;
      thetaStdDev=AprilTagVisionConstants.limelightConstants.thetaMultiTagStdDev;
    }

    swerve.addVision(new VisionObservation(getBotPoseMG2(), getPoseTimestampMG2(), VecBuilder.fill(xyStdDev, xyStdDev, thetaStdDev)));
    

  }



  public Pose2d getBotPoseMG2(){
    return inputs.botPoseMG2;
  }

  /**
   * tells the limelight what the rotation of the gyro is, for determining pose ambiguity stuff
   */
  public void SetRobotOrientation(Rotation2d gyro){
    io.setRobotOrientationMG2(gyro);
  }

  /**
   * 
   * @param tags anything NOT in here will be thrownOut
   */
  public void setPermittedTags(int[] tags){
      io.setPermittedTags(tags);
  }

  /**
   * 
   * 
   * 
   */
  public boolean getPoseValidMG2(Rotation2d gyro){
    
    //get raw data from limelight pose estimator
    Pose2d botpose = inputs.botPoseMG2;
    double diff = 0;

    double gyroval=gyro.getDegrees();
    gyroval = gyroval % (360);

    double x = botpose.getX();
    double y = botpose.getY();

    double tagDist = inputs.avgTagDist;

    //debugging purposes only
    Logger.recordOutput("LLDebug/"+inputs.name+" avgTagDist", tagDist);
    Logger.recordOutput("LLDebug/"+inputs.name+" tagCount", inputs.tagCount);
    Logger.recordOutput("LLDebug/"+inputs.name+" x val", x);
    Logger.recordOutput("LLDebug/"+inputs.name+" y val", y);
    Logger.recordOutput("LLDebug/"+inputs.name+" rdiff", diff);


    // this determines if the raw data from the limelight is valid
    // sometimes the limelight will give really bad data, so we want to throw this out
    // and not use it in our pose estimation.
    // to check for this, we check to see if the rotation from the pose matches
    // the rotation that the gyro is reporting
    // we then check if the pose is actually within the bounds of the field
    // if all these requirements are met, then we can trust the measurement
    // otherwise we ignore it.

    if( 
        (diff<AprilTagVisionConstants.limelightConstants.rotationTolerance) && 
        (tagDist<AprilTagVisionConstants.limelightConstants.throwoutDist) &&
        (botpose.getTranslation().getX() > 0) &&
        (botpose.getTranslation().getX() < AprilTagVisionFieldConstants.fieldLength) &&
        (botpose.getTranslation().getY() > 0) &&
        (botpose.getTranslation().getY() < AprilTagVisionFieldConstants.fieldWidth)) {
          
          return true;
      } else{
          return false;
      }


  }

  public Pose3d getBotPose3d() {
    Pose3d pose = inputs.botPose3d;
    Logger.recordOutput("Limelight" + inputs.name + "/Pose3d", pose);
    return pose;

  }

  // public double getDistToTag() {
  //   //indicies don't match documentation with targetpose_robotspace
  //   Logger.recordOutput("Limelight" + inputs.name + "/distanceToTagHypot", Math.hypot(LimelightHelpers.getCameraPose_TargetSpace(inputs.name)[0], LimelightHelpers.getCameraPose_TargetSpace(inputs.name)[2]));
  //   return Math.hypot(LimelightHelpers.getCameraPose_TargetSpace(inputs.name)[0], LimelightHelpers.getCameraPose_TargetSpace(inputs.name)[2]); // 0 is x, 2 is z 
    
  // }


  public double getPoseTimestampMG2() {
    return inputs.timestamp;
  }

   public String getLimelightName(){
    return inputs.name;
   }

  // angle target is from the center of the limelights crosshair
  public Rotation2d getTX() {
    return Rotation2d.fromDegrees(inputs.tx);
  }

  public double getTA() {
    return inputs.ta;
  }

  public void setPriorityID(int tagID) {
    io.setPriorityID(tagID);
  }

  public double tagCount(){
    return inputs.tagCount;
  }
  

}
