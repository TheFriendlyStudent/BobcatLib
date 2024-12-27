package BobcatLib.Subsystems.Swerve.SimpleSwerve.Swerve.Module.SteerMotor;

import BobcatLib.Logging.Alert;
import BobcatLib.Subsystems.Swerve.SimpleSwerve.Swerve.Module.Utility.ModuleConstants;
import BobcatLib.Subsystems.Swerve.SimpleSwerve.Swerve.Module.parser.ModuleLimitsJson;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.ClosedLoopConfig.ClosedLoopSlot;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

public class VortexSteerMotor implements SteerWrapper {
  private SparkFlex motor;
  private RelativeEncoder encoder;
  /** An {@link Alert} for if the CAN ID is greater than 40. */
  public static final Alert canIdWarning =
      new Alert(
          "JSON",
          "CAN IDs greater than 40 can cause undefined behaviour, please use a CAN ID below 40!",
          Alert.AlertType.WARNING);

  public ModuleConstants chosenModule;
  public ModuleLimitsJson limits;
  private SparkMaxConfig motorConfig;

  private SparkClosedLoopController closedLoopController;

  public VortexSteerMotor(int id, ModuleConstants chosenModule, ModuleLimitsJson limits) {
    if (id >= 40) {
      canIdWarning.set(true);
    }
    this.chosenModule = chosenModule;
    this.limits = limits;
    motor = new SparkFlex(id, MotorType.kBrushless);
    encoder = motor.getEncoder();
    configAngleMotor();
    motor.configure(motorConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
  }

  public void configAngleMotor() {
    /*
     * Create a new SPARK MAX configuration object. This will store the
     * configuration parameters for the SPARK MAX that we will set below.
     */
    motorConfig = new SparkMaxConfig();

    /* Motor Inverts and Neutral Mode */
    motorConfig.encoder.inverted(chosenModule.angleMotorInvert.asREV());

    IdleMode motorMode = chosenModule.angleNeutralMode.asIdleMode();
    motorConfig.idleMode(motorMode);

    /* Current Limiting */
    motorConfig.smartCurrentLimit(chosenModule.json.angleCurrentLimit);

    /* PID Config */
    motorConfig
        .closedLoop
        .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
        .p(chosenModule.anglePID.kP, ClosedLoopSlot.kSlot1)
        .i(chosenModule.anglePID.kI, ClosedLoopSlot.kSlot1)
        .d(chosenModule.anglePID.kD, ClosedLoopSlot.kSlot1)
        .velocityFF(0);

    /* Open and Closed Loop Ramping */
    motorConfig.closedLoopRampRate(chosenModule.json.closedLoopRamp);
    motorConfig.openLoopRampRate(chosenModule.json.openLoopRamp);
  }

  /*
   * sets the angle of the motor by setting the internal pid.
   *
   * @param rotations
   */
  public void setAngle(double rotations) {
    closedLoopController.setReference(rotations, ControlType.kPosition, 0);
  }

  public double getPosition() {
    return encoder.getPosition();
  }

  /**
   * sets the position of the steer motor given the desired rotations.
   *
   * @param absolutePosition
   */
  public void setPosition(double absolutePosition) {
    encoder.setPosition(absolutePosition);
  }

  /** Stops the motors properly. */
  public void stopMotor() {
    motor.stopMotor();
  }
}
