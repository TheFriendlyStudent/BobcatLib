package BobcatLib.Subsystems.Elevators.Utility.Parser;
/**
 * Represents the PID configuration for an elevator system. This class defines the proportional,
 * integral, derivative, and feedforward gains used for controlling the elevator motor.
 */
public class ElevatorPIDJson {

  /**
   * The proportional gain (Kp) for the elevator motor's PID controller. This parameter determines
   * how aggressively the controller reacts to the error.
   */
  public double driveKP = 0.05;

  /**
   * The integral gain (Ki) for the elevator motor's PID controller. This parameter accumulates past
   * errors to address steady-state errors.
   */
  public double driveKI = 0.0;

  /**
   * The derivative gain (Kd) for the elevator motor's PID controller. This parameter reacts to the
   * rate of change of the error to reduce overshoot.
   */
  public double driveKD = 0.0;

  /**
   * The feedforward gain (Kf) for the elevator motor's PID controller. This parameter provides a
   * baseline output to the motor based on expected conditions.
   */
  public double driveKF = 0.0;
}
