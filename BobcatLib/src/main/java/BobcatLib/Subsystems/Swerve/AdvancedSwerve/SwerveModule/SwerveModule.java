package BobcatLib.Subsystems.Swerve.AdvancedSwerve.SwerveModule;

import BobcatLib.Subsystems.Swerve.AdvancedSwerve.Constants.SwerveConstants;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import org.littletonrobotics.junction.Logger;

public class SwerveModule {
  private final SwerveModuleIO io;
  private final SwerveModuleIOInputsAutoLogged inputs = new SwerveModuleIOInputsAutoLogged();

  public final int index;

  private SwerveModuleState desiredState = new SwerveModuleState();

  private SwerveModulePosition[] odometryPositions = new SwerveModulePosition[] {};
  private SwerveConstants constants;

  private SimpleMotorFeedforward driveFeedforward;
  private PIDController driveController;
  private PIDController angleController;

  public SwerveModule(SwerveModuleIO io, int index, SwerveConstants constants) {
    this.io = io;
    this.index = index;
    this.constants = constants;

    driveFeedforward =
        new SimpleMotorFeedforward(
            constants.pidConfigs.driveMotorConfig.kS,
            constants.pidConfigs.driveMotorConfig.kV,
            constants.pidConfigs.driveMotorConfig.kA);
    driveController =
        new PIDController(
            constants.pidConfigs.driveMotorConfig.kP,
            constants.pidConfigs.driveMotorConfig.kI,
            constants.pidConfigs.driveMotorConfig.kD);
    angleController =
        new PIDController(
            constants.pidConfigs.angleMotorConfig.kP,
            constants.pidConfigs.angleMotorConfig.kI,
            constants.pidConfigs.angleMotorConfig.kD);

    angleController.enableContinuousInput(0, 2 * Math.PI);
  }

  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Swerve/Module" + Integer.toString(index), inputs);

    // Calculate positions for odometry
    int sampleCount = inputs.odometryTimestamps.length; // All signals are sampled together
    odometryPositions = new SwerveModulePosition[sampleCount];
    for (int i = 0; i < sampleCount; i++) {
      double positionMeters =
          inputs.odometryDrivePositionsRad[i]
              * (constants.kinematicsConstants.wheelCircumference / (2 * Math.PI));
      Rotation2d angle =
          inputs.odometryAnglePositions[i].minus(
              inputs.offset != null ? inputs.offset : new Rotation2d());
      odometryPositions[i] = new SwerveModulePosition(positionMeters, angle);
    }
  }

  /**
   * Sets the swerve module to the desired state
   *
   * @param state the desired state of the swerve module
   * @return the optimized swerve module state that it was set to
   */
  public SwerveModuleState setDesiredState(SwerveModuleState state) {
    // Optimize the angle for the shortest path
    SwerveModuleState optimizedState = state;
    optimizedState.optimize(getAngle());
    optimizedState.cosineScale(getAngle());

    // if our desired speed is under 1%, maintain current heading, helps with jitter
    Rotation2d angle =
        (Math.abs(desiredState.speedMetersPerSecond)
                <= (constants.speedLimits.moduleLimits.maxVelocity * 0.01))
            ? getAngle()
            : optimizedState.angle;

    io.setAnglePosition(angle);

    double velocityRotPerSec =
        optimizedState.speedMetersPerSecond / constants.kinematicsConstants.wheelCircumference;

    io.setDriveVelocity(Rotation2d.fromRotations(velocityRotPerSec));

    desiredState = optimizedState;
    return optimizedState;
  }

  /** Stops the drive and angle motors */
  public void stop() {
    io.stopAngle();
    io.stopDrive();
  }

  /**
   * Sets the neutral mode of the angle motor
   *
   * @param mode the mode to set it to
   */
  public void setAngleNeutralMode(NeutralModeValue mode) {
    io.setAngleNeutralMode(mode);
  }

  /**
   * Sets the neutral mode of the drive motor
   *
   * @param mode the mode to set it to
   */
  public void setDriveNeutralMode(NeutralModeValue mode) {
    io.setDriveNeutralMode(mode);
  }

  /**
   * Gets the current angle of the swerve module from the CANcoder
   *
   * @return the angle of the module
   */
  public Rotation2d getAngle() {
    return Rotation2d.fromRotations(inputs.canCoderPositionRot);
  }

  /**
   * Gets the current position of the drive motor in meters
   *
   * @return drive motor position, in meters
   */
  public double getPositionMeters() {
    return inputs.drivePositionRot * constants.kinematicsConstants.wheelCircumference;
  }

  /**
   * Gets the current velocity of the drive motor in meters per second
   *
   * @return velocity, in meter per second
   */
  public double getVelocityMetersPerSec() {
    return inputs.driveVelocityRotPerSec * constants.kinematicsConstants.wheelCircumference;
  }

  /**
   * Gets the current position of the swerve module
   *
   * @return the swerve module position
   */
  public SwerveModulePosition getPosition() {
    return new SwerveModulePosition(getPositionMeters(), getAngle());
  }

  /**
   * Gets the current state of the swerve module
   *
   * @return the swerve module state
   */
  public SwerveModuleState getState() {
    return new SwerveModuleState(getVelocityMetersPerSec(), getAngle());
  }

  /**
   * Gets the current desired state that the swerve module has been set to
   *
   * @return the desired state
   */
  public SwerveModuleState getDesiredState() {
    return desiredState;
  }

  /**
   * Gets the raw value of the CANcoder, before the offset is applied. Used only for SmartDashboard
   *
   * @return CANcoder position, in degrees
   */
  public double getRawCanCoder() {
    return inputs.canCoderPositionDeg;
  }

  /** Returns the module positions received this cycle. */
  public SwerveModulePosition[] getOdometryPositions() {
    return odometryPositions;
  }

  /** Returns the timestamps of the samples received this cycle. */
  public double[] getOdometryTimestamps() {
    return inputs.odometryTimestamps;
  }

  public double getDriveAcceleration() {
    return inputs.driveAccelerationRadPerSecSquared;
  }

  public void runCharachterization(double volts) {
    io.runCharachterization(volts);
  }

  public double getVoltage() {
    return inputs.appliedDriveVoltage;
  }
}
