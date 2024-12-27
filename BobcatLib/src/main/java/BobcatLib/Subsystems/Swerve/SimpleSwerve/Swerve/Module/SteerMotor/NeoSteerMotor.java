package BobcatLib.Subsystems.Swerve.SimpleSwerve.Swerve.Module.SteerMotor;

import BobcatLib.Logging.Alert;
import BobcatLib.Subsystems.Swerve.SimpleSwerve.Swerve.Module.Utility.ModuleConstants;
import BobcatLib.Subsystems.Swerve.SimpleSwerve.Swerve.Module.parser.ModuleLimitsJson;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.FaultID;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import edu.wpi.first.wpilibj.DriverStation;

public class NeoSteerMotor implements SteerWrapper {
  private CANSparkFlex motor;
  private RelativeEncoder encoder;
  /** An {@link Alert} for if the CAN ID is greater than 40. */
  public static final Alert canIdWarning =
      new Alert(
          "JSON",
          "CAN IDs greater than 40 can cause undefined behaviour, please use a CAN ID below 40!",
          Alert.AlertType.WARNING);

  public ModuleConstants chosenModule;
  public ModuleLimitsJson limits;

  public NeoSteerMotor(int id, ModuleConstants chosenModule, ModuleLimitsJson limits) {
    if (id >= 40) {
      canIdWarning.set(true);
    }
    this.chosenModule = chosenModule;
    this.limits = limits;
    motor = new CANSparkFlex(id, MotorType.kBrushless);
    encoder = motor.getEncoder();
    configAngleMotor();
    motor.burnFlash();
  }

  public void configAngleMotor() {
    /** Swerve Drive Motor Configuration */
    motor.restoreFactoryDefaults();

    /* Motor Inverts and Neutral Mode */
    motor.setInverted(chosenModule.angleMotorInvert.asREV());
    IdleMode motorMode = chosenModule.angleNeutralMode.asIdleMode();
    motor.setIdleMode(motorMode);

    /* Gear Ratio Config */

    /* Current Limiting */
    motor.setSmartCurrentLimit(chosenModule.json.angleCurrentLimit);

    /* PID Config */
    SparkPIDController controller = motor.getPIDController();
    controller.setP(chosenModule.anglePID.kP, 0);
    controller.setI(chosenModule.anglePID.kI, 0);
    controller.setD(chosenModule.anglePID.kD, 0);
    controller.setFF(0, 0);

    /* Open and Closed Loop Ramping */
    motor.setClosedLoopRampRate(chosenModule.json.closedLoopRamp);
    motor.setOpenLoopRampRate(chosenModule.json.openLoopRamp);

    /* Misc. Configs */
    motor.enableVoltageCompensation(12);
  }

  /*
   * sets the angle of the motor by setting the internal pid.
   *
   * @param rotations
   */
  public void setAngle(double rotations) {
    SparkPIDController controller = motor.getPIDController();
    controller.setReference(rotations, ControlType.kPosition, 0);
    if (motor.getFault(FaultID.kSensorFault)) {
      DriverStation.reportWarning(
          "Sensor Fault on Intake Pivot Motor ID:" + motor.getDeviceId(), false);
    }
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
