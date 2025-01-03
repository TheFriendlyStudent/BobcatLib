package BobcatLib.Subsystems.Swerve.AdvancedSwerve.SwerveModule;

import BobcatLib.Subsystems.Swerve.AdvancedSwerve.Constants.SwerveConstants;
import BobcatLib.Subsystems.Swerve.AdvancedSwerve.Constants.SwerveConstants.ModuleConfig;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
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

  private final DutyCycleOut driveRequest;
  private final DutyCycleOut angleRequest;
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

    driveRequest = new DutyCycleOut(0.0).withEnableFOC(constants.useFOC);
    angleRequest = new DutyCycleOut(0.0).withEnableFOC(constants.useFOC);
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
    inputs.rawCanCoderPositionDeg =
        Rotation2d.fromRotations(angleEncoder.getAbsolutePosition().getValueAsDouble())
            .getDegrees(); // Used only for shuffleboard to display values to get offsets
  }

  /**
   * Sets the percent out of the drive motor
   *
   * @param percent percent to set it to, from -1.0 to 1.0
   */
  public void setDrivePercentOut(double percent) {
    driveMotor.setControl(driveRequest.withOutput(percent));
  }

  /** Stops the drive motor */
  public void stopDrive() {
    driveMotor.stopMotor();
  }

  /**
   * Sets the neutral mode of the drive motor
   *
   * @param mode mode to set it to
   */
  public void setDriveNeutralMode(NeutralModeValue mode) {
    driveMotor.setNeutralMode(mode);
  }

  /**
   * Sets the percent out of the angle motor
   *
   * @param percent percent to set it to, from -1.0 to 1.0
   */
  public void setAnglePercentOut(double percent) {
    angleMotor.setControl(angleRequest.withOutput(percent));
  }

  /** Stops the angle motor */
  public void stopAngle() {
    angleMotor.stopMotor();
  }

  /**
   * Sets the neutral mode of the angle motor
   *
   * @param mode mode to set it to
   */
  public void setAngleNeutralMode(NeutralModeValue mode) {
    angleMotor.setNeutralMode(mode);
  }

  /** Applies all configurations to the drive motor */
  public void configDriveMotor() {
    TalonFXConfiguration config = new TalonFXConfiguration();
    driveMotor.getConfigurator().apply(new TalonFXConfiguration());

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
