// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package BobcatLib.Team177.PoseEstimation;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveWheelPositions;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;

/**
 * This is a wrapper of our custom swerve estimator, which allows for logging of pure odometry,
 * without vision updates, this is particularly useful for tuning swerve state std devs
 *
 * <p>This class wraps {@link BaseBobcatSwerveEstimator BaseBobcaSwervetEstimator}, which wraps
 * {@link BobcatEstimator} to fuse latency-compensated vision measurements with swerve drive encoder
 * distance measurements. It is intended to be a drop-in replacement for {@link
 * edu.wpi.first.math.kinematics.SwerveDriveOdometry}.
 *
 * <p>{@link SwerveDrivePoseEstimator#update} should be called every robot loop.
 *
 * <p>{@link SwerveDrivePoseEstimator#addVisionMeasurement} can be called as infrequently as you
 * want; if you never call it, then this class will behave as regular encoder odometry.
 */
public class BobcatSwerveEstimator extends BaseBobcatSwerveEstimator {
  private BaseBobcatSwerveEstimator odometryTracker;

  /**
   * Constructs a SwerveDrivePoseEstimator with default standard deviations for the model and vision
   * measurements.
   *
   * <p>The default standard deviations of the model states are 0.1 meters for x, 0.1 meters for y,
   * and 0.1 radians for heading. The default standard deviations of the vision measurements are 0.9
   * meters for x, 0.9 meters for y, and 0.9 radians for heading.
   *
   * @param kinematics A correctly-configured kinematics object for your drivetrain.
   * @param gyroAngle The current gyro angle.
   * @param modulePositions The current distance measurements and rotations of the swerve modules.
   * @param initialPoseMeters The starting pose estimate.
   */
  public BobcatSwerveEstimator(
      SwerveDriveKinematics kinematics,
      Rotation2d gyroAngle,
      SwerveModulePosition[] modulePositions,
      Pose2d initialPoseMeters) {
    super(kinematics, gyroAngle, modulePositions, initialPoseMeters);
    odometryTracker =
        new BaseBobcatSwerveEstimator(kinematics, gyroAngle, modulePositions, initialPoseMeters);
  }

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
  public BobcatSwerveEstimator(
      SwerveDriveKinematics kinematics,
      Rotation2d gyroAngle,
      SwerveModulePosition[] modulePositions,
      Pose2d initialPoseMeters,
      Matrix<N3, N1> stateStdDevs,
      Matrix<N3, N1> visionMeasurementStdDevs) {
    super(
        kinematics,
        gyroAngle,
        modulePositions,
        initialPoseMeters,
        stateStdDevs,
        visionMeasurementStdDevs);
    odometryTracker =
        new BaseBobcatSwerveEstimator(kinematics, gyroAngle, modulePositions, initialPoseMeters);
  }

  /**
   * Updates the pose estimator with wheel encoder and gyro information. This should be called every
   * loop.
   *
   * @param gyroAngle The current gyro angle.
   * @param modulePositions The current distance measurements and rotations of the swerve modules.
   * @return The estimated pose of the robot in meters.
   */
  @Override
  public Pose2d update(Rotation2d gyroAngle, SwerveModulePosition[] modulePositions) {
    odometryTracker.update(gyroAngle, modulePositions);
    return super.update(gyroAngle, new SwerveDriveWheelPositions(modulePositions));
  }

  public Pose2d update(
      Rotation2d gyroAngle, SwerveModulePosition[] modulePositions, Matrix<N3, N1> stateStdDevs) {
    setStateStdDevs(stateStdDevs);
    return update(gyroAngle, new SwerveDriveWheelPositions(modulePositions));
  }

  /**
   * Updates the pose estimator with wheel encoder and gyro information. This should be called every
   * loop.
   *
   * @param currentTimeSeconds Time at which this method was called, in seconds.
   * @param gyroAngle The current gyroscope angle.
   * @param modulePositions The current distance measurements and rotations of the swerve modules.
   * @return The estimated pose of the robot in meters.
   */
  @Override
  public Pose2d updateWithTime(
      double currentTimeSeconds, Rotation2d gyroAngle, SwerveModulePosition[] modulePositions) {
    odometryTracker.updateWithTime(currentTimeSeconds, gyroAngle, modulePositions);
    return super.updateWithTime(currentTimeSeconds, gyroAngle, modulePositions);
  }

  /**
   * Updates the pose estimator with wheel encoder and gyro information. This should be called every
   * loop.
   *
   * @param currentTimeSeconds Time at which this method was called, in seconds.
   * @param gyroAngle The current gyroscope angle.
   * @param modulePositions The current distance measurements and rotations of the swerve modules.
   * @return The estimated pose of the robot in meters.
   */
  @Override
  public Pose2d updateWithTime(
      double currentTimeSeconds,
      Rotation2d gyroAngle,
      SwerveModulePosition[] modulePositions,
      Matrix<N3, N1> stateStdDevs) {
    odometryTracker.updateWithTime(currentTimeSeconds, gyroAngle, modulePositions, stateStdDevs);
    return super.updateWithTime(currentTimeSeconds, gyroAngle, modulePositions, stateStdDevs);
  }

  @Override
  public Pose2d updateWithTime(
      double currentTimeSeconds, Rotation2d gyroAngle, SwerveDriveWheelPositions wheelPositions) {
    odometryTracker.updateWithTime(currentTimeSeconds, gyroAngle, wheelPositions);
    return super.updateWithTime(currentTimeSeconds, gyroAngle, wheelPositions);
  }

  public Pose2d getPureOdometry() {
    return odometryTracker.getEstimatedPosition();
  }
}
