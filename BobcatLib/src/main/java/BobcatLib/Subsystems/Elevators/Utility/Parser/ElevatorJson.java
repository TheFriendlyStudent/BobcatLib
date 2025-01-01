package BobcatLib.Subsystems.Elevators.Utility.Parser;

/**
 * Represents the JSON structure used for configuring an elevator system. This class provides nested
 * structures to define the elevator's main attributes, PID configurations, motor settings, and
 * operational limits.
 */
public class ElevatorJson {

  /** Represents the main attributes of the elevator. */
  public class ElevatorMainJson {
    /** The name of the elevator system. */
    public String name = "";
  }

  /** Represents the operational limits of the elevator. */
  public class ElevatorLimitsJson {
    /** The upper limit for the elevator's position. */
    public double upperLimits = 0;

    /** The lower limit for the elevator's position. */
    public double lowerLimits = 0;
  }

  /** An instance of {@link ElevatorMainJson}, representing the elevator's main configuration. */
  public ElevatorMainJson elevator = new ElevatorMainJson();

  /**
   * An instance of {@link ElevatorPIDJson}, representing the PID configuration for the elevator.
   */
  public ElevatorPIDJson elevatorPID = new ElevatorPIDJson();

  /**
   * An instance of {@link ElevatorMotorJson}, representing the motor configuration for the
   * elevator.
   */
  public ElevatorMotorJson motor = new ElevatorMotorJson();

  /**
   * An instance of {@link ElevatorLimitsJson}, representing the operational limits of the elevator.
   */
  public ElevatorLimitsJson limits = new ElevatorLimitsJson();
}
