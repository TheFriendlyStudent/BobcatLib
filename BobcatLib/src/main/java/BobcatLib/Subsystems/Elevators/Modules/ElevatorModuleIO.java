package BobcatLib.Subsystems.Elevators.Modules;

import edu.wpi.first.math.geometry.Rotation2d;

public interface ElevatorModuleIO {
    
  /**
   * Sets the elevator motor output to a specified percentage.
   *
   * @param percent The desired output percentage, where 1.0 represents full forward power and -1.0
   *     represents full reverse power.
   */
  public  default void setPercentOut(double percent) {
  }

  /**
   * Moves the elevator to the specified position.
   *
   * @param position The desired position of the elevator as a {@link Rotation2d} object.
   */
  public default void moveElevator(Rotation2d position) {
  }

  public default void moveElevatorToNext() {
  }
  public default void moveElevatorToPrevious() {
  }

  public default void holdPosition() {
  }

  /** Stops the elevator motor immediately. */
  public default void stop() {
  }

  public default Rotation2d getPosition() {
    return new Rotation2d();
}

public default Rotation2d getCurrentSetPoint() {
  return new Rotation2d();
}
}
