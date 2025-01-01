package BobcatLib.Hardware.Motors;

import BobcatLib.Hardware.Motors.MotorIO.MotorIOInputs;
import BobcatLib.Hardware.Motors.Utility.SoftwareLimitWrapper;
import edu.wpi.first.math.geometry.Rotation2d;

/**
 * The BaseMotor class serves as a wrapper for motor functionality, providing methods for
 * controlling and monitoring motor operations. It interacts with a MotorIO implementation for
 * hardware abstraction.
 */
public class BaseMotor {

  private final MotorIO io;
  private final MotorIOInputs inputs = new MotorIOInputs();
  private SoftwareLimitWrapper limits;

  /**
   * Constructs a new BaseMotor instance.
   *
   * @param io The MotorIO implementation to be used for motor control and feedback.
   * @param limits A SoftwareLimitWrapper instance to handle motor software limits.
   */
  public BaseMotor(MotorIO io, SoftwareLimitWrapper limits) {
    this.limits = limits;
    this.io = io;
  }

  /**
   * Periodically updates the motor inputs and processes them. This method should be called
   * frequently to ensure the motor's state is up-to-date.
   */
  public void periodic() {
    io.updateInputs(inputs);
  }

  /**
   * Retrieves the current position of the motor as a Rotation2d object.
   *
   * @return The motor's position as a Rotation2d instance.
   */
  public Rotation2d getPosition() {
    return io.getPosition();
  }

  /**
   * Retrieves the current velocity of the motor.
   *
   * @return The motor's velocity in meters per second.
   */
  public double getVelocity() {
    return io.getVelocity();
  }

  /** Stops the motor and performs a fault check to ensure no issues are present. */
  public void stopMotor() {
    io.stopMotor();
    checkForFaults();
  }

  /**
   * Sets the motor's speed.
   *
   * @param speedInMPS The desired speed in meters per second.
   * @param mechanismCircumference The circumference of the mechanism connected to the motor.
   * @param isOpenLoop Whether the speed control should be open-loop.
   */
  public void setSpeed(double speedInMPS, double mechanismCircumference, boolean isOpenLoop) {
    io.setSpeed(speedInMPS, mechanismCircumference, isOpenLoop);
    checkForFaults();
  }

  /**
   * Sets the motor's angle in rotations.
   *
   * @param angleInRotations The desired angle in rotations.
   */
  public void setAngle(double angleInRotations) {
    io.setAngle(angleInRotations);
    checkForFaults();
  }

  /**
   * Sets the motor control voltage for SysID mode.
   *
   * @param volts The control voltage to apply to the motor.
   */
  public void setControl(double volts) {
    io.setControl(volts);
    checkForFaults();
  }

  /** Performs a fault check on the motor to detect and handle any issues. */
  public void checkForFaults() {
    io.checkForFaults();
  }

  /**
   * Retrieves the MotorIO implementation associated with this BaseMotor.
   *
   * @return The MotorIO instance being used for motor control and feedback.
   */
  public MotorIO getMotor() {
    return io;
  }

  /**
   * Retrieves the CAN ID of the motor.
   *
   * @return The motor's CAN ID.
   */
  public int getCanId() {
    return io.getCanId();
  }

  /**
   * Configures the motor with both lower and upper software limits.
   *
   * @param lower The lower limit as a Rotation2d instance.
   * @param upper The upper limit as a Rotation2d instance.
   * @return The updated BaseMotor instance.
   */
  public BaseMotor withLimits(Rotation2d lower, Rotation2d upper) {
    limits =
        new SoftwareLimitWrapper(
            lower.getRotations(),
            upper.getRotations(),
            SoftwareLimitWrapper.SoftwareLimitType.BOTH);
    return this;
  }

  /**
   * Configures the motor with an upper software limit.
   *
   * @param rotations The upper limit as a Rotation2d instance.
   * @return The updated BaseMotor instance.
   */
  public BaseMotor withUpperLimit(Rotation2d rotations) {
    limits =
        new SoftwareLimitWrapper(
            rotations.getRotations(), SoftwareLimitWrapper.SoftwareLimitType.UPPER);
    return this;
  }

  /**
   * Configures the motor with a lower software limit.
   *
   * @param rotations The lower limit as a Rotation2d instance.
   * @return The updated BaseMotor instance.
   */
  public BaseMotor withLowerLimit(Rotation2d rotations) {
    limits =
        new SoftwareLimitWrapper(
            rotations.getRotations(), SoftwareLimitWrapper.SoftwareLimitType.LOWER);
    return this;
  }
}
