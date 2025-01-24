package BobcatLib.Subsystems.Swerve.SimpleSwerve.Swerve.Module.SteerMotor;

import BobcatLib.Logging.Alert;
import BobcatLib.Subsystems.Swerve.SimpleSwerve.Swerve.Module.Utility.ModuleConstants;
import BobcatLib.Subsystems.Swerve.SimpleSwerve.Swerve.Module.parser.ModuleLimitsJson;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

public class KrakenSteerMotor implements SteerWrapper {
  private TalonFX mAngleMotor;
  private TalonFXConfiguration swerveAngleFXConfig = new TalonFXConfiguration();
  /* angle motor control requests */
  private final PositionVoltage anglePosition = new PositionVoltage(0);
  /** An {@link Alert} for if the CAN ID is greater than 40. */
  public static final Alert canIdWarning =
      new Alert(
          "JSON",
          "CAN IDs greater than 40 can cause undefined behaviour, please use a CAN ID below 40!",
          Alert.AlertType.WARNING);

  private ModuleConstants chosenModule;
  public ModuleLimitsJson limits;

  public KrakenSteerMotor(
      int id, ModuleConstants chosenModule, String canivorename, ModuleLimitsJson limits) {
    if (id >= 40) {
      canIdWarning.set(true);
    }
    this.chosenModule = chosenModule;
    this.limits = limits;

    if (canivorename == "" || canivorename == "rio") {
      mAngleMotor = new TalonFX(id);
    } else {
      mAngleMotor = new TalonFX(id, canivorename);
    }

    configAngleMotor();
    mAngleMotor.getConfigurator().apply(swerveAngleFXConfig);
  }

  public void configAngleMotor() {
    /** Swerve Angle Motor Configurations */
    /* Motor Inverts and Neutral Mode */
    swerveAngleFXConfig.MotorOutput.Inverted = chosenModule.angleMotorInvert.asCTRE();
    swerveAngleFXConfig.MotorOutput.NeutralMode =
        chosenModule.angleNeutralMode.asNeutralModeValue();

    /* Gear Ratio and Wrapping Config */

    swerveAngleFXConfig.Feedback.SensorToMechanismRatio = chosenModule.angleGearRatio;
    swerveAngleFXConfig.ClosedLoopGeneral.ContinuousWrap = true;

    /* Current Limiting */
    swerveAngleFXConfig.CurrentLimits.SupplyCurrentLimitEnable =
        chosenModule.json.angleEnableCurrentLimit;
    swerveAngleFXConfig.CurrentLimits.SupplyCurrentLimit = chosenModule.json.angleCurrentLimit;

    /* PID Config */
    swerveAngleFXConfig.Slot0.kP = chosenModule.anglePID.kP;
    swerveAngleFXConfig.Slot0.kI = chosenModule.anglePID.kI;
    swerveAngleFXConfig.Slot0.kD = chosenModule.anglePID.kD;

    /* Open and Closed Loop Ramping */
    swerveAngleFXConfig.ClosedLoopRamps.withVoltageClosedLoopRampPeriod(
        chosenModule.json.closedLoopRamp);
    swerveAngleFXConfig.OpenLoopRamps.withVoltageOpenLoopRampPeriod(chosenModule.json.openLoopRamp);
  }

  /*
   * sets the angle of the motor by setting the internal pid.
   *
   * @param rotations
   */
  public void setAngle(double rotations) {
    mAngleMotor.setControl(anglePosition.withPosition(rotations));
  }

  /**
   * gets the position of the steer motor.
   *
   * @return position of the steer motor in rotations.
   */
  public double getPosition() {
    return mAngleMotor.getPosition().getValueAsDouble();
  }

  /**
   * sets the position of the steer motor given the desired rotations.
   *
   * @param absolutePosition
   */
  public void setPosition(double absolutePosition) {
    mAngleMotor.setPosition(absolutePosition);
  }

  /** Stops the motors properly. */
  public void stopMotor() {
    mAngleMotor.stopMotor();
  }
}
