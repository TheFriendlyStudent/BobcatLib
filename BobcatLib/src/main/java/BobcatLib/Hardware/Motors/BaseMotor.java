package BobcatLib.Hardware.Motors;

import BobcatLib.Hardware.Motors.MotorIO.MotorIOInputs;
import edu.wpi.first.math.geometry.Rotation2d;

/**
 * The BaseMotor class serves as a wrapper for motor functionality, providing methods for
 * controlling and monitoring motor operations. It interacts with a MotorIO implementation for
 * hardware abstraction.
 */
public class BaseMotor {

  private final MotorIO io;
  private final MotorIOInputs inputs = new MotorIOInputs();

  /**
   * Constructs a new BaseMotor instance.
   *
   * @param io The MotorIO implementation to be used for motor control and feedback.
   */
  public BaseMotor(MotorIO io) {
    this.io = io;
  }

  /** Periodically updates the motor inputs and processes them using the Logger. */
  public void periodic() {
    io.updateInputs(inputs);
  }

  /**
   * Retrieves the current position of the motor as a Rotation2d object.
   *
   * @return The motor's position.
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

  /** Stops the motor and performs a fault check. */
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
   * @param volts The control voltage to apply.
   */
  public void setControl(double volts) {
    io.setControl(volts);
    checkForFaults();
  }

  /** Performs a fault check on the motor. */
  public void checkForFaults() {
    io.checkForFaults();
  }

  /**
   * Retrieves the MotorIO implementation associated with this BaseMotor.
   *
   * @return The MotorIO instance.
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
}
