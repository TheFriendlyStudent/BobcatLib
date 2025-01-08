package BobcatLib.Subsystems.Swerve.AdvancedSwerve.SwerveModule;

import BobcatLib.Subsystems.Swerve.AdvancedSwerve.Constants.SwerveConstants;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

/**
 * Physics sim implementation of module IO. The sim models are configured using a set of module
 * constants from Phoenix. Simulation is always based on voltage control.
 */
public class SwerveModuleIOSim implements SwerveModuleIO {
  // TunerConstants doesn't support separate sim constants, so they are declared locally
  private final double DRIVE_KP;
  private final double DRIVE_KD;
  private final double DRIVE_KS;
  private final double DRIVE_KV_ROT; // Same units as TunerConstants: (volt * secs) / rotation
  private final double DRIVE_KV;
  private final double TURN_KP;
  private final double TURN_KD;
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

  public SwerveModuleIOSim(SwerveConstants constants) {
    DRIVE_KP = constants.pidConfigs.driveMotorConfig.kP;
    DRIVE_KD = constants.pidConfigs.driveMotorConfig.kD;
    DRIVE_KS = constants.pidConfigs.driveMotorConfig.kS;
    DRIVE_KV_ROT =
        constants.pidConfigs.driveMotorConfig.kV; // TODO sanity check this when tuning 2025
    DRIVE_KV = 1.0 / Units.rotationsToRadians(1.0 / DRIVE_KV_ROT);
    TURN_KP = constants.pidConfigs.angleMotorConfig.kP;
    TURN_KD = constants.pidConfigs.angleMotorConfig.kD;
    driveController = new PIDController(DRIVE_KP, 0, DRIVE_KD);
    turnController = new PIDController(TURN_KP, 0, TURN_KD);

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
    // Run closed-loop control
    driveAppliedVolts =
        driveFFVolts + driveController.calculate(driveSim.getAngularVelocityRadPerSec());

    turnAppliedVolts = turnController.calculate(turnSim.getAngularPositionRad());

    // Update simulation state
    driveSim.setInputVoltage(MathUtil.clamp(driveAppliedVolts, -12.0, 12.0));
    turnSim.setInputVoltage(MathUtil.clamp(turnAppliedVolts, -12.0, 12.0));
    driveSim.update(0.02);
    turnSim.update(0.02);

    // Update drive inputs
    inputs.drivePositionRot = driveSim.getAngularPositionRotations();
    inputs.driveVelocityRotPerSec = driveSim.getAngularVelocityRPM() / 60.0;
    inputs.appliedDriveVoltage = driveAppliedVolts;
    inputs.driveCurrentAmps = Math.abs(driveSim.getCurrentDrawAmps());

    // Update turn inputs
    inputs.canCoderPositionDeg = new Rotation2d(turnSim.getAngularPositionRad()).getDegrees();
    inputs.canCoderPositionRot = new Rotation2d(turnSim.getAngularPositionRad()).getRotations();
    inputs.turnAngularVelocityRadPerSec = turnSim.getAngularVelocityRadPerSec();
    inputs.angleAppliedVoltage = turnAppliedVolts;
    inputs.angleCurrentAmps = Math.abs(turnSim.getCurrentDrawAmps());

    inputs.odometryTimestamps = new double[] {Timer.getFPGATimestamp()};
    inputs.odometryDrivePositionsRad = new double[] {inputs.drivePositionRot * 2 * Math.PI};
    inputs.odometryAnglePositions =
        new Rotation2d[] {Rotation2d.fromRotations(inputs.canCoderPositionRot)};
  }

  /** velocity in units/second */
  @Override
  public void setDriveVelocity(Rotation2d angularVelocity) {
    double velocityRadPerSec = angularVelocity.getRadians();
    driveFFVolts = DRIVE_KS * Math.signum(velocityRadPerSec) + DRIVE_KV * velocityRadPerSec;
    driveController.setSetpoint(velocityRadPerSec);
  }

  @Override
  public void setAnglePosition(Rotation2d rotation) {
    turnController.setSetpoint(rotation.getRadians());
  }
}
