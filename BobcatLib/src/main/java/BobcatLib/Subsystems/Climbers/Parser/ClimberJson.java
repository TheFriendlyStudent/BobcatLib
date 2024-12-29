package BobcatLib.Subsystems.Climbers.Parser;

/**
 * Represents the configuration and settings for a climber mechanism.
 *
 * <p>This class is intended to store the climber's properties, limits, and control parameters in a
 * structured and serializable format (e.g., JSON).
 */
public class ClimberJson {
  /** The name of the climber mechanism. */
  public String name = "";

  /** The upper limit of the climber's motion, in rotations or units. */
  public double upperLimit = 0.00;

  /** The lower limit of the climber's motion, in rotations or units. */
  public double lowerLimit = 0.00;

  /**
   * Indicates whether the climber motor direction is inverted. If true, the motor runs in the
   * opposite direction of the default configuration.
   */
  public boolean inverted = false;

  /**
   * Indicates whether the climber motor is in brake mode.
   *
   * <p>When true, the motor resists motion when power is not applied.
   */
  public boolean climberMotorBreakMode = true;

  public int motorId = 0;

  public double kP = 0.0;
}
