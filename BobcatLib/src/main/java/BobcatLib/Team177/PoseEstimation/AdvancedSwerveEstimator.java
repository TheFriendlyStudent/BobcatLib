// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package BobcatLib.Team177.PoseEstimation;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;

/**
 * This class wraps {@link SwerveDriveOdometry Swerve Drive Odometry} to fuse latency-compensated
 * vision measurements with swerve drive encoder distance measurements. It is intended to be a
 * drop-in replacement for {@link SwerveDriveOdometry}.
 *
 * <p>{@link AdvancedSwerveEstimator#update} should be called every robot loop.
 *
 * <p>{@link AdvancedSwerveEstimator#addVisionMeasurement} can be called as infrequently as you
 * want; if you never call it, then this class will behave as regular encoder odometry.
 */
public class AdvancedSwerveEstimator extends AdvancedPoseEstimator<SwerveModulePosition[]> {
  private final int m_numModules;
  private double distrustAccel;
  private double throwoutAccel;


  /**
   * Constructs a SwerveDrivePoseEstimator.
   *
   * @param kinematics A correctly-configured kinematics object for your drivetrain.
   * @param gyroAngle The current gyro angle.
   * @param modulePositions The current distance and rotation measurements of the swerve modules.
   * @param initialPoseMeters The starting pose estimate.
   * @param stateStdDevs Standard deviations of the pose estimate (x position in meters, y position
   *     in meters, and heading in radians). Increase these numbers to trust your state estimate
   *     less.
   * @param visionMeasurementStdDevs Standard deviations of the vision pose measurement (x position
   *     in meters, y position in meters, and heading in radians). Increase these numbers to trust
   *     the vision pose measurement less.
   */
  public AdvancedSwerveEstimator(
      SwerveDriveKinematics kinematics,
      Rotation2d gyroAngle,
      SwerveModulePosition[] modulePositions,
      Pose2d initialPoseMeters,
      Matrix<N3, N1> stateStdDevs,
      Matrix<N3, N1> visionMeasurementStdDevs,
      double distrustAcceleration,
      double throwoutAcceleration) {
    super(
        kinematics,
        new SwerveDriveOdometry(kinematics, gyroAngle, modulePositions, initialPoseMeters),
        stateStdDevs,
        visionMeasurementStdDevs);

    distrustAccel = distrustAcceleration;
    throwoutAccel = throwoutAcceleration;
    m_numModules = modulePositions.length;
  }

  @Override
  public Pose2d updateWithTime(
      double currentTimeSeconds, Rotation2d gyroAngle, SwerveModulePosition[] wheelPositions) {
    if (wheelPositions.length != m_numModules) {
      throw new IllegalArgumentException(
          "Number of modules is not consistent with number of wheel locations provided in "
              + "constructor");
    }

    return super.updateWithTime(currentTimeSeconds, gyroAngle, wheelPositions);
  }

  public OdometryTrustState getTrustState(double acceleration){
    if(acceleration > throwoutAccel){
        return OdometryTrustState.THROWOUT;
    }else if(acceleration > distrustAccel){
        return OdometryTrustState.DISTRUST;
    }else{
        return OdometryTrustState.TRUST;
    }
  }
  public OdometryTrustState getTrustState(double acceleration, double throwoutAcceleration, double distrustAcceleration){
    if(acceleration > throwoutAcceleration){
        return OdometryTrustState.THROWOUT;
    }else if(acceleration > distrustAcceleration){
        return OdometryTrustState.DISTRUST;
    }else{
        return OdometryTrustState.TRUST;
    }
  }
}

enum OdometryTrustState{
    TRUST,
    DISTRUST,
    THROWOUT
}
