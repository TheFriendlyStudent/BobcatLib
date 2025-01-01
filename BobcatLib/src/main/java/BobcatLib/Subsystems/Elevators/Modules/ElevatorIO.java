package BobcatLib.Subsystems.Elevators.Modules;

import BobcatLib.Subsystems.Elevators.Utility.Parser.ElevatorJson;
import edu.wpi.first.math.geometry.Rotation2d;
import org.littletonrobotics.junction.AutoLog;

/**
 * Interface representing the Input/Output operations for an Elevator system. Provides methods for
 * configuring, controlling, and monitoring the elevator's state.
 */
public interface ElevatorIO {

  /**
   * Class for storing the inputs related to the elevator. Annotated with autologging for logging
   * support.
   */
  @AutoLog
  public static class ElevatorIOInputs {
    /** The current position of the elevator motor as a double . */
    public double motorPosition = 0;
  }

  /**
   * Configures the elevator with default settings. This method is optional to override in
   * implementations.
   */
  public default void configElevator() {}

  /**
   * Loads configuration settings for the elevator from a JSON file.
   *
   * @param json The {@link ElevatorJson} instance containing the configuration data.
   */
  public default void loadConfigurationFromFile(ElevatorJson json) {}

  /**
   * Updates the elevator's inputs with the provided values.
   *
   * @param inputs The {@link ElevatorIOInputs} instance containing updated input values.
   */
  public default void updateInputs(ElevatorIOInputs inputs) {}

  /**
   * Sets the elevator motor output as a percentage of the maximum power.
   *
   * @param percent The desired percentage output (-1.0 to 1.0).
   */
  public default void setPercentOut(double percent) {}

  /** Holds the elevator's current position by applying necessary control logic. */
  public default void holdPos() {}

  /** Stops the elevator motor by setting its output to zero. */
  public default void stop() {}

  /**
   * Retrieves the current position of the elevator.
   *
   * @return The current position as a {@link Rotation2d} object.
   */
  public default Rotation2d getPosition() {
    return new Rotation2d();
  }

  /**
   * Sets the elevator to a specified position.
   *
   * @param position The target position as a {@link Rotation2d} object.
   */
  public default void setPosition(Rotation2d position) {}
}
