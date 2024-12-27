package BobcatLib.Hardware.Motors;

import BobcatLib.Hardware.Motors.SensorHelpers.InvertedWrapper;
import BobcatLib.Hardware.Motors.SensorHelpers.NeutralModeWrapper;
import BobcatLib.Logging.FaultsAndErrors.SparkMaxFaults;
import BobcatLib.Utilities.CANDeviceDetails;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;

/**
 * The VortexMotor class implements the MotorIO interface and provides an abstraction for
 * controlling a Vortex motor. It handles motor initialization, configuration, and control
 * operations such as setting speed, angle, and performing fault checks.
 */
public class VortexMotor implements MotorIO {
  private int motorCanId = 0;
  private final SimpleMotorFeedforward motorFeedFordward;
  private CANSparkFlex mMotor;
  private RelativeEncoder encoder;
  private String busName = "";
  private SparkMaxFaults faults;
  private CANDeviceDetails details;

  /**
   * Constructs a VortexMotor instance with the specified CAN Details, bus name, and configuration.
   *
   * @param details The CAN details of the motor.
   * @param busname The bus name where the motor is connected.
   * @param config The motor configuration settings.
   */
  public VortexMotor(CANDeviceDetails details, String busname, MotorConfigs config) {
    this.details = details;
    int id = details.getDeviceNumber();
    this.busName = busname;
    motorCanId = id;
    double motorKS = 0.00;
    double motorKV = 0.00;
    double motorKA = 0.00;
    motorFeedFordward = new SimpleMotorFeedforward(motorKS, motorKV, motorKA);
    /* Drive Motor Config */
    mMotor = new CANSparkFlex(id, MotorType.kBrushless);
    encoder = mMotor.getEncoder();
    configMotor(config);
    mMotor.burnFlash();
    encoder.setPosition(0.0);
    faults = new SparkMaxFaults(motorCanId);
  }

  /**
   * Configures the motor settings based on the provided MotorConfigs.
   *
   * @param cfg The MotorConfigs object containing configuration parameters.
   */
  public void configMotor(MotorConfigs cfg) {
    /** Motor Configuration */
    mMotor.restoreFactoryDefaults();

    /* Motor Inverts and Neutral Mode */
    mMotor.setInverted(new InvertedWrapper(cfg.isInverted).asREV());
    mMotor.setIdleMode(new NeutralModeWrapper(cfg.mode).asIdleMode());

    /* Gear Ratio Config */
    encoder.setVelocityConversionFactor(cfg.optionalRev.driveConversionVelocityFactor);
    encoder.setPositionConversionFactor(cfg.optionalRev.driveConversionPositionFactor);

    /* Current Limiting */
    mMotor.setSmartCurrentLimit(cfg.optionalRev.SupplyCurrentLimit);

    /* PID Config */
    SparkPIDController controller = mMotor.getPIDController();
    controller.setP(cfg.kP);
    controller.setI(cfg.kI);
    controller.setD(cfg.kD);
    controller.setFF(0);

    /* Open and Closed Loop Ramping */
    mMotor.setClosedLoopRampRate(cfg.optionalRev.closedLoopRamp);
    mMotor.setOpenLoopRampRate(cfg.optionalRev.openLoopRamp);

    /* Misc. Configs */
    mMotor.enableVoltageCompensation(12);
  }

  /**
   * Updates the inputs for the motor with the latest sensor data.
   *
   * @param inputs The MotorIOInputs object to update.
   */
  public void updateInputs(MotorIOInputs inputs) {
    inputs.motorPosition = getPosition();
    inputs.motorVelocity = getVelocity();
  }

  /**
   * Retrieves the current position of the motor.
   *
   * @return The motor position as a Rotation2d object.
   */
  public Rotation2d getPosition() {
    double pos = encoder.getPosition();
    return Rotation2d.fromRotations(pos);
  }

  /**
   * Retrieves the current velocity of the motor.
   *
   * @return The motor velocity in meters per second.
   */
  public double getVelocity() {
    return encoder.getVelocity();
  }

  /**
   * Sets the motor speed based on the desired speed in meters per second.
   *
   * @param speedInMPS The desired speed in meters per second.
   */
  public void setSpeed(double speedInMPS) {
    double output = motorFeedFordward.calculate(speedInMPS);
    SparkPIDController controller = mMotor.getPIDController();
    controller.setReference(output, ControlType.kVelocity, 0);
  }

  /**
   * Sets the motor speed based on the desired speed and mechanism circumference.
   *
   * @param speedInMPS The desired speed in meters per second.
   * @param mechanismCircumference The circumference of the mechanism connected to the motor.
   */
  public void setSpeed(double speedInMPS, double mechanismCircumference) {
    double velocity = speedInMPS / mechanismCircumference;
    double output = motorFeedFordward.calculate(velocity);
    SparkPIDController controller = mMotor.getPIDController();
    controller.setReference(output, ControlType.kVelocity, 0);
  }

  /**
   * Sets the motor angle to the specified value in rotations.
   *
   * @param angleInRotations The desired angle in rotations.
   */
  public void setAngle(double angleInRotations) {
    SparkPIDController controller = mMotor.getPIDController();
    controller.setReference(angleInRotations, ControlType.kPosition, 0);
  }

  /**
   * Sets the motor control voltage.
   *
   * @param volts The voltage to apply to the motor, capped at 12V.
   */
  public void setControl(double volts) {
    if (volts > 12) {
      volts = 12;
    }
    double output = volts / 12;
    mMotor.set(output);
  }

  /** Stops the motor. */
  public void stopMotor() {
    mMotor.stopMotor();
  }

  /** Checks for any faults in the motor. */
  public void checkForFaults() {
    faults.hasFaultOccured();
  }

  /**
   * Retrieves the CANSparkFlex motor instance.
   *
   * @return The CANSparkFlex instance representing the motor.
   */
  public CANSparkFlex getMotor() {
    return mMotor;
  }

  /**
   * Retrieves the CAN ID of the motor.
   *
   * @return The CAN ID of the motor.
   */
  public int getCanId() {
    return motorCanId;
  }
}
