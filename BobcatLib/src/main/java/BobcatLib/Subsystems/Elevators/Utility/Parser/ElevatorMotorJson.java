package BobcatLib.Subsystems.Elevators.Utility.Parser;

/**
 * Represents the configuration for an elevator motor, including its type, ID, CAN bus, and
 * inversion state.
 */
public class ElevatorMotorJson {

  /** Default constructor for creating an empty {@code ElevatorMotorJson} instance. */
  public ElevatorMotorJson() {}

  /**
   * Constructs an {@code ElevatorMotorJson} instance with the specified motor ID and CAN bus name.
   *
   * @param id The CAN ID or pin ID of the motor.
   * @param canbus The name of the CAN bus where the motor resides.
   */
  public ElevatorMotorJson(int id, String canbus) {
    this.id = id;
    this.canbus = canbus;
  }

  /** The type of motor, e.g., Kraken, Falcon, Vortex, or NEO. */
  public String motor_type = "";

  /** The CAN ID or pin ID of the motor. */
  public int id = 0;

  /** The name of the CAN bus where the motor resides, if applicable. */
  public String canbus = "";

  /** Indicates whether the motor's direction is inverted. */
  public boolean inverted = false;
}
