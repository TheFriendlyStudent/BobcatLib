package BobcatLib.Subsystems.Swerve.SimpleSwerve.Swerve.Module.Utility.Pose;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;

public class WpiPoseEstimator implements PoseLib {
  public final SwerveDrivePoseEstimator swerveDrivePoseEstimator;

  public WpiPoseEstimator(
      SwerveDriveKinematics swerveKinematics,
      Rotation2d yaw,
      SwerveModulePosition[] modulePositions,
      Pose2d initPose) {
    swerveDrivePoseEstimator =
        new SwerveDrivePoseEstimator(swerveKinematics, yaw, modulePositions, initPose);
  }

  /**
   * Gets the estimated robot pose.
   *
   * @return The estimated robot pose in meters.
   */
  public Pose2d getEstimatedPosition() {
    return swerveDrivePoseEstimator.getEstimatedPosition();
  }
  ;

  /**
   * Resets the robot's position on the field.
   *
   * <p>The gyroscope angle does not need to be reset in the user's robot code. The library
   * automatically takes care of offsetting the gyro angle.
   *
   * @param gyroAngle The angle reported by the gyroscope.
   * @param modulePositions The current distance measurements and rotations of the swerve modules.
   * @param poseMeters The position on the field that your robot is at.
   */
  public void resetPosition(
      Rotation2d gyroAngle, SwerveModulePosition[] modulePositions, Pose2d poseMeters) {
    swerveDrivePoseEstimator.resetPosition(gyroAngle, modulePositions, poseMeters);
  }

  public Pose2d updateWithTime(
      double currentTimeSeconds, Rotation2d gyroAngle, SwerveModulePosition[] wheelPositions) {
    return new Pose2d();
  }

  public Pose2d updateWithTime(
      double currentTimeSeconds,
      Rotation2d gyroAngle,
      SwerveModulePosition[] wheelPositions,
      Matrix<N3, N1> deviations) {
    return new Pose2d();
  }

  public Pose2d update(Rotation2d gyroAngle, SwerveModulePosition[] wheelPositions) {
    return new Pose2d();
  }

  public Pose2d update(
      Rotation2d gyroAngle, SwerveModulePosition[] wheelPositions, Matrix<N3, N1> deviations) {
    return new Pose2d();
  }

  public void addVisionMeasurement(
      Pose2d botPoseMG2, double poseTimestampMG2, Matrix<N3, N1> stdDev) {}
}
