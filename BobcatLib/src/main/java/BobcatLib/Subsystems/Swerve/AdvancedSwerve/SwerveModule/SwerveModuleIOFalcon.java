package BobcatLib.Subsystems.Swerve.AdvancedSwerve.SwerveModule;

import BobcatLib.Subsystems.Swerve.AdvancedSwerve.Constants.SwerveConstants.SwerveMotorConfig;
import BobcatLib.Subsystems.Swerve.AdvancedSwerve.PhoenixOdometryThread;
import BobcatLib.Team254.ModuleConstants;
import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.AbsoluteSensorRangeValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import java.util.Queue;

public class SwerveModuleIOFalcon implements SwerveModuleIO {
  private final TalonFX driveMotor;
  private final TalonFX angleMotor;
  private final CANcoder angleEncoder;

  private final Rotation2d encoderOffset;

  private final DutyCycleOut driveRequest;
  private final DutyCycleOut angleRequest;

  private final StatusSignal<Double> internalTempDrive;
  private final StatusSignal<Double> processorTempDrive;
  private final StatusSignal<Double> internalTempAngle;
  private final StatusSignal<Double> processorTempAngle;
  private final StatusSignal<Double> driveAppliedVolts;

  private final Queue<Double> timestampQueue;

  private final Queue<Double> drivePositionQueue;
  private final StatusSignal<Double> drivePosition;
  private final StatusSignal<Double> driveVelocity;
  private final StatusSignal<Double> driveAcceleration;

  private final Queue<Double> anglePositionQueue;
  private final StatusSignal<Double> angleAbsolutePosition;
  private String canbus;
  private VoltageOut sysidControl = new VoltageOut(0);
  private double driveGearRatio;
  private double angleGearRatio;
  public SwerveMotorConfig driveMotorConfig;
  public SwerveMotorConfig angleMotorConfig;
  AbsoluteSensorRangeValue cancoderSensorRange;
  SensorDirectionValue cancoderSensorDirection;
  private final PhoenixOdometryThread thread;

  public SwerveModuleIOFalcon(
      ModuleConstants moduleConstants,
      boolean useFOC,
      SwerveMotorConfig driveMotorConfig,
      SwerveMotorConfig angleMotorConfig,
      AbsoluteSensorRangeValue cancoderSensorRange,
      SensorDirectionValue cancoderSensorDirection,
      String canbus,
      PhoenixOdometryThread thread) {
    this.thread = thread;
    encoderOffset = moduleConstants.angleOffset;
    this.cancoderSensorDirection = cancoderSensorDirection;
    this.cancoderSensorRange = cancoderSensorRange;
    this.canbus = canbus;
    driveGearRatio = driveMotorConfig.gearRatio;
    angleGearRatio = angleMotorConfig.gearRatio;
    this.driveMotorConfig = driveMotorConfig;
    this.angleMotorConfig = angleMotorConfig;

    angleEncoder = new CANcoder(moduleConstants.cancoderID, canbus);
    configAngleEncoder();
    angleMotor = new TalonFX(moduleConstants.angleMotorID, canbus);
    configAngleMotor();
    driveMotor = new TalonFX(moduleConstants.driveMotorID, canbus);
    configDriveMotor();

    driveRequest = new DutyCycleOut(0.0).withEnableFOC(useFOC);
    angleRequest = new DutyCycleOut(0.0).withEnableFOC(useFOC);

    timestampQueue = thread.makeTimestampQueue();

    drivePosition = driveMotor.getPosition();
    drivePositionQueue = thread.registerSignal(driveMotor, driveMotor.getPosition());
    driveVelocity = driveMotor.getVelocity();
    driveAcceleration = driveMotor.getAcceleration();
    angleAbsolutePosition = angleEncoder.getAbsolutePosition();
    anglePositionQueue = thread.registerSignal(angleEncoder, angleEncoder.getPosition());

    internalTempDrive = driveMotor.getDeviceTemp();
    processorTempDrive = driveMotor.getProcessorTemp();
    internalTempAngle = angleMotor.getDeviceTemp();
    processorTempAngle = angleMotor.getProcessorTemp();
    driveAppliedVolts = driveMotor.getMotorVoltage();

    BaseStatusSignal.setUpdateFrequencyForAll(
        50.0, drivePosition, driveVelocity, angleAbsolutePosition, driveAppliedVolts);
    BaseStatusSignal.setUpdateFrequencyForAll(
        5, internalTempAngle, internalTempDrive, processorTempAngle, processorTempDrive);
    driveMotor.optimizeBusUtilization();
    angleMotor.optimizeBusUtilization();
  }

  public void updateInputs(SwerveModuleIOInputs inputs) {
    inputs.offset = encoderOffset;
    BaseStatusSignal.refreshAll(
        drivePosition,
        driveVelocity,
        angleAbsolutePosition,
        internalTempAngle,
        internalTempDrive,
        processorTempAngle,
        processorTempDrive,
        driveAcceleration,
        driveAppliedVolts);

    inputs.drivePositionRot = drivePosition.getValueAsDouble() / driveGearRatio;
    inputs.driveVelocityRotPerSec = driveVelocity.getValueAsDouble() / driveGearRatio;

    inputs.canCoderPositionRot =
        Rotation2d.fromRadians(
                MathUtil.angleModulus(
                    Rotation2d.fromRotations(angleAbsolutePosition.getValueAsDouble())
                        .minus(encoderOffset)
                        .getRadians()))
            .getRotations();
    inputs.rawCanCoderPositionDeg =
        Rotation2d.fromRotations(angleAbsolutePosition.getValueAsDouble())
            .getDegrees(); // Used only for shuffleboard to display values to get offsets

    inputs.odometryTimestamps =
        timestampQueue.stream().mapToDouble((Double value) -> value).toArray();
    inputs.odometryDrivePositionsRad =
        drivePositionQueue.stream()
            .mapToDouble((Double value) -> Units.rotationsToRadians(value) / driveGearRatio)
            .toArray();
    inputs.odometryAnglePositions =
        anglePositionQueue.stream()
            // .map((Double value) ->
            // Rotation2d.fromRadians(MathUtil.angleModulus(Rotation2d.fromRotations(value).minus(encoderOffset).getRadians())))
            .map((Double value) -> Rotation2d.fromRotations(value))
            .toArray(Rotation2d[]::new);

    timestampQueue.clear();
    drivePositionQueue.clear();
    anglePositionQueue.clear();

    inputs.internalTempAngle = internalTempAngle.getValueAsDouble();
    inputs.internalTempDrive = internalTempDrive.getValueAsDouble();
    inputs.processorTempAngle = processorTempAngle.getValueAsDouble();
    inputs.processorTempDrive = processorTempDrive.getValueAsDouble();
    inputs.appliedDriveVoltage = driveAppliedVolts.getValueAsDouble();
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

    config.CurrentLimits.SupplyCurrentLimitEnable = driveMotorConfig.supplyCurrentLimitEnable;
    config.CurrentLimits.SupplyCurrentLimit = driveMotorConfig.supplyCurrentLimit;
    config.CurrentLimits.SupplyCurrentThreshold = driveMotorConfig.supplyCurrentThreshold;
    config.CurrentLimits.SupplyTimeThreshold = driveMotorConfig.supplyTimeThreshold;

    config.CurrentLimits.StatorCurrentLimitEnable = driveMotorConfig.statorCurrentLimitEnable;
    config.CurrentLimits.StatorCurrentLimit = driveMotorConfig.statorCurrentLimit;

    config.OpenLoopRamps.DutyCycleOpenLoopRampPeriod = driveMotorConfig.openLoopRamp;
    config.OpenLoopRamps.TorqueOpenLoopRampPeriod = driveMotorConfig.openLoopRamp;
    config.OpenLoopRamps.VoltageOpenLoopRampPeriod = driveMotorConfig.openLoopRamp;
    config.ClosedLoopRamps.DutyCycleClosedLoopRampPeriod = driveMotorConfig.closedLoopRamp;
    config.ClosedLoopRamps.TorqueClosedLoopRampPeriod = driveMotorConfig.closedLoopRamp;
    config.ClosedLoopRamps.VoltageClosedLoopRampPeriod = driveMotorConfig.closedLoopRamp;

    config.MotorOutput.Inverted = driveMotorConfig.motorInvert;
    config.MotorOutput.NeutralMode = driveMotorConfig.neutralMode;

    driveMotor.getConfigurator().apply(config);

    driveMotor.getConfigurator().setPosition(0);
  }

  /** Applies all configurations to the angle motor */
  public void configAngleMotor() {
    TalonFXConfiguration config = new TalonFXConfiguration();
    angleMotor.getConfigurator().apply(new TalonFXConfiguration());

    config.CurrentLimits.SupplyCurrentLimitEnable = angleMotorConfig.supplyCurrentLimitEnable;
    config.CurrentLimits.SupplyCurrentThreshold = angleMotorConfig.supplyCurrentThreshold;
    config.CurrentLimits.SupplyTimeThreshold = angleMotorConfig.supplyTimeThreshold;

    config.CurrentLimits.StatorCurrentLimitEnable = driveMotorConfig.statorCurrentLimitEnable;
    config.CurrentLimits.StatorCurrentLimit = driveMotorConfig.statorCurrentLimit;

    config.MotorOutput.Inverted = angleMotorConfig.motorInvert;
    config.MotorOutput.NeutralMode = angleMotorConfig.neutralMode;

    angleMotor.getConfigurator().apply(config);
  }

  /** Applies all configurations to the angle absolute encoder */
  public void configAngleEncoder() {
    CANcoderConfiguration config = new CANcoderConfiguration();
    angleEncoder.getConfigurator().apply(new CANcoderConfiguration());

    config.MagnetSensor.AbsoluteSensorRange = cancoderSensorRange;
    config.MagnetSensor.SensorDirection = cancoderSensorDirection;

    angleEncoder.getConfigurator().apply(config);
  }

  @Override
  public void runCharachterization(double volts) {
    sysidControl.withOutput(volts);
    angleMotor.setPosition(0);
    driveMotor.setControl(sysidControl);
  }
}
