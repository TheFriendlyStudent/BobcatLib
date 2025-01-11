package BobcatLib.Subsystems.Swerve.AdvancedSwerve.SwerveModule;

import BobcatLib.Subsystems.Swerve.AdvancedSwerve.Constants.SwerveConstants;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

/**
 * Physics sim implementation of module IO. The sim models are configured using a set of module
 * constants from Phoenix. Simulation is always based on voltage control.
 */
public class SwerveModuleIOSim implements SwerveModuleIO {
  // TunerConstants doesn't support separate sim constants, so they are declared locally
  private SimpleMotorFeedforward driveFeedforward;
  private final DCMotor DRIVE_GEARBOX = DCMotor.getKrakenX60Foc(1);
  private final DCMotor TURN_GEARBOX = DCMotor.getKrakenX60Foc(1);

  private final DCMotorSim driveSim;
  private final DCMotorSim turnSim;

  private PIDController driveController;
  private PIDController turnController;
  private double driveFFVolts = 0.0;
  private double driveAppliedVolts = 0.0;
  private double turnAppliedVolts = 0.0;

  private static final double driveInertia = 0.0001; // kg m^2
  private static final double steerInertia = 0.025; // kg m^2
  private static final double driveMotorGearRatio = 1.0 / 8.16;
  private static final double steerMotorGearRatio = 1.0 / 12.8; // sample values

  private String index;

  public SwerveModuleIOSim(SwerveConstants constants, int index) {
    this.index = Integer.toString(index);
    driveFeedforward =
        new SimpleMotorFeedforward(
            constants.pidConfigs.driveMotorConfig.kS,
            constants.pidConfigs.driveMotorConfig.kV,
            constants.pidConfigs.driveMotorConfig.kA);

    driveController =
        new PIDController(
            constants.pidConfigs.angleMotorConfig.kP, 0, constants.pidConfigs.angleMotorConfig.kD);
    turnController =
        new PIDController(
            constants.pidConfigs.angleMotorConfig.kP, 0, constants.pidConfigs.angleMotorConfig.kD);
    turnController.enableContinuousInput(0, 2 * Math.PI);

    // Create drive and turn sim models
    driveSim =
        new DCMotorSim(
            LinearSystemId.createDCMotorSystem(DRIVE_GEARBOX, driveInertia, driveMotorGearRatio),
            DRIVE_GEARBOX);
    turnSim =
        new DCMotorSim(
            LinearSystemId.createDCMotorSystem(TURN_GEARBOX, steerInertia, steerMotorGearRatio),
            TURN_GEARBOX);

    // Enable wrapping for turn PID
    turnController.enableContinuousInput(-Math.PI, Math.PI);
  }

  @Override
  public void updateInputs(SwerveModuleIOInputs inputs) {
    // Update drive inputs
    inputs.drivePositionRot = driveSim.getAngularPositionRotations();
    inputs.driveVelocityRotPerSec = driveSim.getAngularVelocityRPM() / 60.0;

    inputs.driveAccelerationRadPerSecSquared = driveSim.getAngularAccelerationRadPerSecSq();
    inputs.appliedDriveVoltage = driveAppliedVolts;
    inputs.driveCurrentAmps = Math.abs(driveSim.getCurrentDrawAmps());

    // Update turn inputs
    inputs.canCoderPositionRot = turnSim.getAngularPositionRotations();
    inputs.canCoderPositionDeg = 69; // turnSim.getAngularPositionRotations() * 360;
    inputs.turnAngularVelocityRadPerSec = turnSim.getAngularVelocityRadPerSec();
    inputs.angleAppliedVoltage = turnAppliedVolts;
    inputs.angleCurrentAmps = Math.abs(turnSim.getCurrentDrawAmps());

    // Update temperature inputs (example values, replace with actual simulation data if available)
    inputs.internalTempDrive = 40.0; // Example value
    inputs.processorTempDrive = 45.0; // Example value
    inputs.internalTempAngle = 35.0; // Example value
    inputs.processorTempAngle = 38.0; // Example value

    // Update odometry inputs
    inputs.odometryTimestamps = new double[] {Timer.getFPGATimestamp()};
    inputs.odometryAnglePositions =
        new Rotation2d[] {Rotation2d.fromDegrees(inputs.canCoderPositionDeg)};
    inputs.odometryDrivePositionsRad = new double[] {driveSim.getAngularPositionRad()};
  }

  /** velocity in units/second */
  @Override
  public void setDriveVelocity(Rotation2d angularVelocity) {
    double velocityRadPerSec = angularVelocity.getRadians();
    driveFFVolts = driveFeedforward.calculate(velocityRadPerSec);
    driveController.setSetpoint(velocityRadPerSec);
  }

  @Override
  public void setAnglePosition(Rotation2d rotation) {
    turnController.setSetpoint(rotation.getRadians());
  }
}
