package BobcatLib.Subsystems.Swerve.AdvancedSwerve.SwerveModule;

import BobcatLib.Subsystems.Swerve.AdvancedSwerve.Constants.SwerveConstants;
import BobcatLib.Subsystems.Swerve.AdvancedSwerve.Constants.SwerveConstants.ModuleConfig;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionTorqueCurrentFOC;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;

public class SwerveModuleIOFalcon implements SwerveModuleIO {
  private final TalonFX driveMotor;
  private final TalonFX angleMotor;
  private final CANcoder angleEncoder;

  private final Rotation2d encoderOffset;

  private final VelocityTorqueCurrentFOC driveRequest; // TODO should we use torquecurrent
  private final PositionTorqueCurrentFOC angleRequest;
  private final SwerveConstants constants;

  public SwerveModuleIOFalcon(ModuleConfig module, SwerveConstants constants) {

    encoderOffset = module.angleOffset;
    this.constants = constants;
    angleEncoder = new CANcoder(module.cancoderID, constants.canbus);
    configAngleEncoder();
    angleMotor = new TalonFX(module.angleMotorID, constants.canbus);
    configAngleMotor();
    driveMotor = new TalonFX(module.driveMotorID, constants.canbus);
    configDriveMotor();

    // Velocity in rot/sec
    driveRequest = new VelocityTorqueCurrentFOC(0);
    // Position in rot
    angleRequest = new PositionTorqueCurrentFOC(0);
  }

  public void updateInputs(SwerveModuleIOInputs inputs) {
    inputs.drivePositionRot =
        driveMotor.getPosition().getValueAsDouble()
            / constants.pidConfigs.driveMotorConfig.gearRatio;
    inputs.driveVelocityRotPerSec =
        driveMotor.getVelocity().getValueAsDouble()
            / constants.pidConfigs.driveMotorConfig.gearRatio;

    inputs.canCoderPositionRot =
        Rotation2d.fromRadians(
                MathUtil.angleModulus(
                    Rotation2d.fromRotations(angleEncoder.getAbsolutePosition().getValueAsDouble())
                        .minus(encoderOffset)
                        .getRadians()))
            .getRotations();
    inputs.canCoderPositionDeg =
        Rotation2d.fromRotations(angleEncoder.getAbsolutePosition().getValueAsDouble())
            .getDegrees(); // Used only for shuffleboard to display values to get offsets
  }

  /**
   * sets the velocity of the drive motor
   *
   * @param velocityRadPerSec velocity to set it to
   */
  @Override
  public void setDriveVelocity(Rotation2d velocityRadPerSec) {
    driveMotor.setControl(driveRequest.withVelocity(velocityRadPerSec.getRotations()));
  }

  /** Stops the drive motor */
  @Override
  public void stopDrive() {
    driveMotor.stopMotor();
  }

  /**
   * Sets the neutral mode of the drive motor
   *
   * @param mode mode to set it to
   */
  @Override
  public void setDriveNeutralMode(NeutralModeValue mode) {
    driveMotor.setNeutralMode(mode);
  }

  /**
   * sets the position of the angle motor
   *
   * @param angle angle to set it to
   */
  @Override
  public void setAnglePosition(Rotation2d angle) {
    angleMotor.setControl(angleRequest.withPosition(angle.getRotations()));
  }

  /** Stops the angle motor */
  @Override
  public void stopAngle() {
    angleMotor.stopMotor();
  }

  /**
   * Sets the neutral mode of the angle motor
   *
   * @param mode mode to set it to
   */
  @Override
  public void setAngleNeutralMode(NeutralModeValue mode) {
    angleMotor.setNeutralMode(mode);
  }

  /** Applies all configurations to the drive motor */
  public void configDriveMotor() {
    TalonFXConfiguration config = new TalonFXConfiguration();
    // apply a blank config to wipe all old ones
    driveMotor.getConfigurator().apply(new TalonFXConfiguration());

    config.Slot0.kP = constants.pidConfigs.driveMotorConfig.kP;
    config.Slot0.kI = constants.pidConfigs.driveMotorConfig.kI;
    config.Slot0.kD = constants.pidConfigs.driveMotorConfig.kD;
    config.Slot0.kS = constants.pidConfigs.driveMotorConfig.kS;
    config.Slot0.kV = constants.pidConfigs.driveMotorConfig.kV;
    config.Slot0.kA = constants.pidConfigs.driveMotorConfig.kA;

    config.CurrentLimits.SupplyCurrentLimitEnable =
        constants.pidConfigs.driveMotorConfig.supplyCurrentLimitEnable;
    config.CurrentLimits.SupplyCurrentLimit =
        constants.pidConfigs.driveMotorConfig.supplyCurrentLimit;
    config.CurrentLimits.SupplyCurrentLowerLimit =
        constants.pidConfigs.driveMotorConfig.supplyCurrentThreshold;
    config.CurrentLimits.SupplyCurrentLowerTime =
        constants.pidConfigs.driveMotorConfig.supplyTimeThreshold;
    config.CurrentLimits.StatorCurrentLimitEnable =
        constants.pidConfigs.driveMotorConfig.statorCurrentLimitEnable;
    config.CurrentLimits.StatorCurrentLimit =
        constants.pidConfigs.driveMotorConfig.statorCurrentLimit;

    config.OpenLoopRamps.DutyCycleOpenLoopRampPeriod =
        constants.pidConfigs.driveMotorConfig.openLoopRamp;
    config.OpenLoopRamps.TorqueOpenLoopRampPeriod =
        constants.pidConfigs.driveMotorConfig.openLoopRamp;
    config.OpenLoopRamps.VoltageOpenLoopRampPeriod =
        constants.pidConfigs.driveMotorConfig.openLoopRamp;
    config.ClosedLoopRamps.DutyCycleClosedLoopRampPeriod =
        constants.pidConfigs.driveMotorConfig.openLoopRamp;
    config.ClosedLoopRamps.TorqueClosedLoopRampPeriod =
        constants.pidConfigs.driveMotorConfig.openLoopRamp;
    config.ClosedLoopRamps.VoltageClosedLoopRampPeriod =
        constants.pidConfigs.driveMotorConfig.openLoopRamp;

    config.MotorOutput.Inverted = constants.driveInverted;
    config.MotorOutput.NeutralMode = constants.angleNeutralMode;

    driveMotor.getConfigurator().apply(config);

    driveMotor.getConfigurator().setPosition(0);
  }

  /** Applies all configurations to the angle motor */
  public void configAngleMotor() {

    TalonFXConfiguration config = new TalonFXConfiguration();
    angleMotor.getConfigurator().apply(new TalonFXConfiguration());

    config.Slot0.kP = constants.pidConfigs.angleMotorConfig.kP;
    config.Slot0.kI = constants.pidConfigs.angleMotorConfig.kI;
    config.Slot0.kD = constants.pidConfigs.angleMotorConfig.kD;
    config.Slot0.kS = constants.pidConfigs.angleMotorConfig.kS;
    config.Slot0.kV = constants.pidConfigs.angleMotorConfig.kV;
    config.Slot0.kA = constants.pidConfigs.angleMotorConfig.kA;

    config.CurrentLimits.SupplyCurrentLimitEnable =
        constants.pidConfigs.angleMotorConfig.supplyCurrentLimitEnable;
    config.CurrentLimits.SupplyCurrentLimit =
        constants.pidConfigs.angleMotorConfig.supplyCurrentLimit;
    config.CurrentLimits.SupplyCurrentLowerLimit =
        constants.pidConfigs.angleMotorConfig.supplyCurrentThreshold;
    config.CurrentLimits.SupplyCurrentLowerTime =
        constants.pidConfigs.angleMotorConfig.supplyTimeThreshold;
    config.CurrentLimits.StatorCurrentLimitEnable =
        constants.pidConfigs.angleMotorConfig.statorCurrentLimitEnable;
    config.CurrentLimits.StatorCurrentLimit =
        constants.pidConfigs.angleMotorConfig.statorCurrentLimit;

    config.OpenLoopRamps.DutyCycleOpenLoopRampPeriod =
        constants.pidConfigs.angleMotorConfig.openLoopRamp;
    config.OpenLoopRamps.TorqueOpenLoopRampPeriod =
        constants.pidConfigs.angleMotorConfig.openLoopRamp;
    config.OpenLoopRamps.VoltageOpenLoopRampPeriod =
        constants.pidConfigs.angleMotorConfig.openLoopRamp;
    config.ClosedLoopRamps.DutyCycleClosedLoopRampPeriod =
        constants.pidConfigs.angleMotorConfig.openLoopRamp;
    config.ClosedLoopRamps.TorqueClosedLoopRampPeriod =
        constants.pidConfigs.angleMotorConfig.openLoopRamp;
    config.ClosedLoopRamps.VoltageClosedLoopRampPeriod =
        constants.pidConfigs.angleMotorConfig.openLoopRamp;

    config.MotorOutput.Inverted = constants.angleInverted;
    config.MotorOutput.NeutralMode = constants.driveNeutralMode;

    angleMotor.getConfigurator().apply(config);
  }

  /** Applies all configurations to the angle absolute encoder */
  public void configAngleEncoder() {
    CANcoderConfiguration config = new CANcoderConfiguration();
    angleEncoder.getConfigurator().apply(new CANcoderConfiguration());

    config.MagnetSensor.SensorDirection = constants.cancoderDirection;

    angleEncoder.getConfigurator().apply(config);
  }
}
