package BobcatLib.Subsystems.Swerve.AdvancedSwerve.Swerve.Module;

import java.util.Queue;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import BobcatLib.Subsystems.Swerve.AdvancedSwerve.Swerve.SwerveConstants;
import BobcatLib.Subsystems.Swerve.AdvancedSwerve.Swerve.Utility.PhoenixOdometryThread;
import BobcatLib.Subsystems.Swerve.AdvancedSwerve.Utility.Team254.ModuleConstants;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;

public class SwerveModuleIOFalcon implements SwerveModuleIO {
    private final TalonFX driveMotor;
    private final TalonFX angleMotor;
    private final CANcoder angleEncoder;

    private final Rotation2d encoderOffset;

    private final DutyCycleOut driveRequest;
    private final DutyCycleOut angleRequest;

    private final StatusSignal<Temperature> internalTempDrive;
    private final StatusSignal<Temperature> processorTempDrive;
    private final StatusSignal<Temperature> internalTempAngle;
    private final StatusSignal<Temperature> processorTempAngle;
    private final StatusSignal<Voltage> driveAppliedVolts;

    private final Queue<Double> timestampQueue;

    private final Queue<Double> drivePositionQueue;
    private final StatusSignal<Angle> drivePosition;
    private final StatusSignal<AngularVelocity> driveVelocity;
    private final StatusSignal<AngularAcceleration> driveAcceleration;

    private final Queue<Double> anglePositionQueue;
    private final StatusSignal<Angle> angleAbsolutePosition;
    private VoltageOut sysidControl = new VoltageOut(0); 


    public SwerveModuleIOFalcon(ModuleConstants moduleConstants) {
        encoderOffset = moduleConstants.angleOffset;

        angleEncoder = new CANcoder(moduleConstants.cancoderID);
        configAngleEncoder();
        angleMotor = new TalonFX(moduleConstants.angleMotorID);
        configAngleMotor();
        driveMotor = new TalonFX(moduleConstants.driveMotorID);
        configDriveMotor();

        driveRequest = new DutyCycleOut(0.0).withEnableFOC(SwerveConstants.useFOC);
        angleRequest = new DutyCycleOut(0.0).withEnableFOC(SwerveConstants.useFOC);

        timestampQueue = PhoenixOdometryThread.getInstance().makeTimestampQueue();

        drivePosition = driveMotor.getPosition();
        drivePositionQueue = PhoenixOdometryThread.getInstance().registerSignal(driveMotor,drivePosition);
        driveVelocity = driveMotor.getVelocity();
        driveAcceleration = driveMotor.getAcceleration();
        angleAbsolutePosition = angleEncoder.getAbsolutePosition();
        anglePositionQueue = PhoenixOdometryThread.getInstance().registerSignal(angleEncoder, angleEncoder.getPosition());

        internalTempDrive = driveMotor.getDeviceTemp();
        processorTempDrive = driveMotor.getProcessorTemp();
        internalTempAngle = angleMotor.getDeviceTemp();
        processorTempAngle = angleMotor.getProcessorTemp();
        driveAppliedVolts = driveMotor.getMotorVoltage();
        

        BaseStatusSignal.setUpdateFrequencyForAll(
            50.0, drivePosition, driveVelocity, angleAbsolutePosition, driveAppliedVolts);
        BaseStatusSignal.setUpdateFrequencyForAll(5, internalTempAngle, internalTempDrive, processorTempAngle, processorTempDrive);
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


        inputs.drivePositionRot = drivePosition.getValueAsDouble() / SwerveConstants.driveGearRatio;
        inputs.driveVelocityRotPerSec = driveVelocity.getValueAsDouble() / SwerveConstants.driveGearRatio;

        inputs.canCoderPositionRot = Rotation2d.fromRadians(MathUtil.angleModulus(Rotation2d.fromRotations(angleAbsolutePosition.getValueAsDouble()).minus(encoderOffset).getRadians())).getRotations();
        inputs.rawCanCoderPositionDeg = Rotation2d.fromRotations(angleAbsolutePosition.getValueAsDouble()).getDegrees(); // Used only for shuffleboard to display values to get offsets

        inputs.odometryTimestamps =
            timestampQueue.stream().mapToDouble((Double value) -> value).toArray();
        inputs.odometryDrivePositionsRad =
            drivePositionQueue.stream()
                .mapToDouble((Double value) -> Units.rotationsToRadians(value) / SwerveConstants.driveGearRatio)
                .toArray();
        inputs.odometryAnglePositions =
            anglePositionQueue.stream()
                // .map((Double value) -> Rotation2d.fromRadians(MathUtil.angleModulus(Rotation2d.fromRotations(value).minus(encoderOffset).getRadians())))
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
     * @param percent percent to set it to, from -1.0 to 1.0
     */
    public void setDrivePercentOut(double percent) {
        driveMotor.setControl(driveRequest.withOutput(percent));
    }

    /**
     * Stops the drive motor
     */
    public void stopDrive() {
        driveMotor.stopMotor();
    }

    /**
     * Sets the neutral mode of the drive motor
     * @param mode mode to set it to
     */
    public void setDriveNeutralMode(NeutralModeValue mode) {
        driveMotor.setNeutralMode(mode);
    }

    /**
     * Sets the percent out of the angle motor
     * @param percent percent to set it to, from -1.0 to 1.0
     */
    public void setAnglePercentOut(double percent) {
        angleMotor.setControl(angleRequest.withOutput(percent));
    }

    /**
     * Stops the angle motor
     */
    public void stopAngle() {
        angleMotor.stopMotor();
    }

    /**
     * Sets the neutral mode of the angle motor
     * @param mode mode to set it to
     */
    public void setAngleNeutralMode(NeutralModeValue mode) {
        angleMotor.setNeutralMode(mode);
    }

    /**
     * Applies all configurations to the drive motor
     */
    public void configDriveMotor() {
        TalonFXConfiguration config = new TalonFXConfiguration();
        driveMotor.getConfigurator().apply(new TalonFXConfiguration());

        config.CurrentLimits.SupplyCurrentLimitEnable = SwerveConstants.Configs.Module.Drive.driveSupplyCurrentLimitEnable;
        config.CurrentLimits.SupplyCurrentLimit = SwerveConstants.Configs.Module.Drive.driveSupplyCurrentLimit;

        config.CurrentLimits.StatorCurrentLimitEnable = SwerveConstants.Configs.Module.Drive.driveStatorCurrentLimitEnable;
        config.CurrentLimits.StatorCurrentLimit = SwerveConstants.Configs.Module.Drive.driveStatorCurrentLimit;

        config.OpenLoopRamps.DutyCycleOpenLoopRampPeriod = SwerveConstants.Configs.Module.Drive.openLoopRamp;
        config.OpenLoopRamps.TorqueOpenLoopRampPeriod = SwerveConstants.Configs.Module.Drive.openLoopRamp;
        config.OpenLoopRamps.VoltageOpenLoopRampPeriod = SwerveConstants.Configs.Module.Drive.openLoopRamp;
        config.ClosedLoopRamps.DutyCycleClosedLoopRampPeriod = SwerveConstants.Configs.Module.Drive.closedLoopRamp;
        config.ClosedLoopRamps.TorqueClosedLoopRampPeriod = SwerveConstants.Configs.Module.Drive.closedLoopRamp;
        config.ClosedLoopRamps.VoltageClosedLoopRampPeriod = SwerveConstants.Configs.Module.Drive.closedLoopRamp;

        

        config.MotorOutput.Inverted = SwerveConstants.Configs.Module.Drive.driveMotorInvert;
        config.MotorOutput.NeutralMode = SwerveConstants.Configs.Module.Drive.driveNeutralMode;

        driveMotor.getConfigurator().apply(config);

        driveMotor.getConfigurator().setPosition(0);
    }

    /**
     * Applies all configurations to the angle motor
     */
    public void configAngleMotor() {
        TalonFXConfiguration config = new TalonFXConfiguration();
        angleMotor.getConfigurator().apply(new TalonFXConfiguration());

        config.CurrentLimits.SupplyCurrentLimitEnable = SwerveConstants.Configs.Module.Angle.angleSupplyCurrentLimitEnable;

        config.CurrentLimits.StatorCurrentLimitEnable = SwerveConstants.Configs.Module.Angle.angleStatorCurrentLimitEnable;
        config.CurrentLimits.StatorCurrentLimit = SwerveConstants.Configs.Module.Angle.angleStatorCurrentLimit;
        

        config.MotorOutput.Inverted = SwerveConstants.Configs.Module.Angle.angleMotorInvert;
        config.MotorOutput.NeutralMode = SwerveConstants.Configs.Module.Angle.angleNeutralMode;

        angleMotor.getConfigurator().apply(config);
    }


    /**
     * Applies all configurations to the angle absolute encoder
     */
    public void configAngleEncoder() {
        CANcoderConfiguration config = new CANcoderConfiguration();
        angleEncoder.getConfigurator().apply(new CANcoderConfiguration());
        config.MagnetSensor.SensorDirection = SwerveConstants.Configs.Module.CANCoder.sensorDirection;

        angleEncoder.getConfigurator().apply(config);
    }

    @Override
    public void runCharachterization(double volts){
        sysidControl.withOutput(volts);
        angleMotor.setPosition(0);
        driveMotor.setControl(sysidControl);
    }
}