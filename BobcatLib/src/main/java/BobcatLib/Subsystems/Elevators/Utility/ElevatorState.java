package BobcatLib.Subsystems.Elevators.Utility;

import edu.wpi.first.math.geometry.Rotation2d;

/**
 * Represents the state of the Elevator system, including its rotations. This class is immutable;
 * once the object is created, its state cannot be changed.
 */
public class ElevatorState {

  /** The current rotations of the Elevator. */
  private final Rotation2d ElevatorRotations;

  /** Default constructor that initializes the Elevator state with a zero rotation. */
  public ElevatorState() {
    this.ElevatorRotations = new Rotation2d();
  }

  /**
   * Constructor that initializes the Elevator state with the provided rotation.
   *
   * @param ElevatorRotations The current rotations of the Elevator.
   */
  public ElevatorState(Rotation2d ElevatorRotations) {
    this.ElevatorRotations = ElevatorRotations;
  }

  /**
   * Gets the current rotation of the Elevator.
   *
   * @return The current Elevator rotations as a Rotation2d object.
   */
  public Rotation2d getElevatorRotations() {
    return ElevatorRotations;
  }

  /**
   * Returns a string representation of the ElevatorState object.
   *
   * @return A string representation of the Elevator state.
   */
  @Override
  public String toString() {
    return "ElevatorState{" + "ElevatorRotations=" + ElevatorRotations + '}';
  }
}
