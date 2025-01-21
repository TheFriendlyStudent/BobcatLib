package BobcatLib.Subsystems.Vision;

import BobcatLib.Subsystems.Swerve.SimpleSwerve.Swerve.Module.Utility.Pose.WpiPoseEstimator;
import BobcatLib.Subsystems.Vision.Components.VisionIO;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * Represents the Intake Subsystem in the robot. It interacts with the intake module to control the
 * intake mechanism, load configurations from a file, and periodically update inputs.
 */
public class VisionSubsystem extends SubsystemBase {
  public VisionIO io;
  public final String name;
  /**
   * Constructor for the VisionSubsystem.
   *
   * @param name The name of the Vision subsystem.
   */
  public VisionSubsystem(String name, VisionIO io) {
    this.name = name;
    this.io = io;
  }

  public void periodic() {
    io.getResult_PoseEstimate();
    io.getResult_Target();
  }

  public void updatePoseEstimator(WpiPoseEstimator wpi) {
    io.updatePoseEstimator(wpi);
  }

  public void updatePoseEstimator(Rotation2d angle, SwerveModulePosition[] positions) {
    io.updatePoseEstimator(angle, positions);
  }

  public boolean TargetDetected() {
    boolean validTarget = false;
    boolean algaeDetected = io.findInstanceOf("algae").isSeen;
    boolean coralDetected = io.findInstanceOf("coral").isSeen;
    if (algaeDetected || coralDetected) {
      validTarget = true;
    }
    return validTarget;
  }

  public Pose2d getTargetPose() {
    return new Pose2d();
  }
}
