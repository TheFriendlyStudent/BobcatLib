package BobcatLib.Subsystems.Swerve.SimpleSwerve.Swerve.Module.DriveMotor;

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
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.DriverStation;

public class NeoDriveMotor implements DriveWrapper {
  private CANSparkFlex motor;
  private RelativeEncoder encoder;
  /* Feed Forward Setup */
  private final SimpleMotorFeedforward driveFeedForward;
  /** An {@link Alert} for if the CAN ID is greater than 40. */
  public static final Alert canIdWarning =
      new Alert(
          "JSON",
          "CAN IDs greater than 40 can cause undefined behaviour, please use a CAN ID below 40!",
          Alert.AlertType.WARNING);

  public ModuleConstants chosenModule;
  public ModuleLimitsJson limits;

  public NeoDriveMotor(int id, ModuleConstants chosenModule, ModuleLimitsJson limits) {
    if (id >= 40) {
      canIdWarning.set(true);
    }
    this.chosenModule = chosenModule;
    this.limits = limits;
    double driveKS = 0.00;
    double driveKV = 0.00;
    double driveKA = 0.0;
    driveFeedForward = new SimpleMotorFeedforward(driveKS, driveKV, driveKA);
    /* Drive Motor Config */
    motor = new CANSparkFlex(id, MotorType.kBrushless);
    encoder = motor.getEncoder();
    configDriveMotor();
    motor.burnFlash();
    encoder.setPosition(0.0);
  }

  public void configDriveMotor() {
    /** Swerve Drive Motor Configuration */
    motor.restoreFactoryDefaults();

    /* Motor Inverts and Neutral Mode */
    motor.setInverted(chosenModule.driveMotorInvert.asREV());
    IdleMode motorMode = chosenModule.driveNeutralMode.asIdleMode();
    motor.setIdleMode(motorMode);

    /* Gear Ratio Config */
    encoder.setVelocityConversionFactor(chosenModule.driveConversionVelocityFactor);
    encoder.setPositionConversionFactor(chosenModule.driveConversionPositionFactor);

    /* Current Limiting */
    motor.setSmartCurrentLimit(chosenModule.json.driveCurrentLimit);

    /* PID Config */
    SparkPIDController controller = motor.getPIDController();
    controller.setP(chosenModule.drivePID.kP);
    controller.setI(chosenModule.drivePID.kI);
    controller.setD(chosenModule.drivePID.kD);
    controller.setFF(0);

    /* Open and Closed Loop Ramping */
    motor.setClosedLoopRampRate(chosenModule.json.closedLoopRamp);
    motor.setOpenLoopRampRate(chosenModule.json.openLoopRamp);

    /* Misc. Configs */
    motor.enableVoltageCompensation(12);
  }

  /**
   * gets the velocity of the drive motor in rotations per second either using percentage output (
   * dutycycleout ) or velocity control triggered by the isOpenLoop parameter
   *
   * @param desiredState
   * @param isOpenLoop
   */
  public void setSpeed(SwerveModuleState desiredState, boolean isOpenLoop) {
    double maxSpeed = limits.maxSpeed;
    if (isOpenLoop) {
      double percentOutput = desiredState.speedMetersPerSecond / maxSpeed;
      motor.set(percentOutput);

    } else {
      double velocity = desiredState.speedMetersPerSecond;
      double output = driveFeedForward.calculate(velocity);
      SparkPIDController controller = motor.getPIDController();
      controller.setReference(output, ControlType.kVelocity, 0);
    }
    if (motor.getFault(FaultID.kSensorFault)) {
      DriverStation.reportWarning("Sensor Fault on Intake Motor ID:" + motor.getDeviceId(), false);
    }
  }

  /**
   * gets the position of the drive motor in rotations
   *
   * @return position
   */
  public double getPosition() {
    return encoder.getPosition();
  }

  /**
   * gets the velocity of the drive motor in rotations per second.
   *
   * @return velocity
   */
  public double getVelocity() {
    return encoder.getVelocity() * 60;
  }

  /** Stops the motors properly. */
  public void stopMotor() {
    motor.stopMotor();
  }
}
