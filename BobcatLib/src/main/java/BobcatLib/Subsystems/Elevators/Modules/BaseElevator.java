package BobcatLib.Subsystems.Elevators.Modules;

import BobcatLib.Subsystems.Elevators.Modules.ElevatorIO.ElevatorIOInputs;
import BobcatLib.Subsystems.Elevators.Utility.ElevatorState;
import edu.wpi.first.math.geometry.Rotation2d;

public class BaseElevator {
  private ElevatorIO io;
  private ElevatorIOInputs inputs = new ElevatorIOInputs();

  public BaseElevator(ElevatorIO io) {
    this.io = io;
  }

  public void periodic() {
    io.updateInputs(inputs);
  }

  /**
   * Sets the elevator motor output to a specified percentage.
   *
   * @param percent The desired output percentage, where 1.0 represents full forward power and -1.0
   *     represents full reverse power.
   */
  public void setPercentOut(double percent) {
    io.setPercentOut(percent);
  }

  /**
   * Moves the elevator to the specified position.
   *
   * @param position The desired position of the elevator as a {@link Rotation2d} object.
   */
  public void moveElevator(Rotation2d position) {
    io.setPosition(position);
  }

  public ElevatorState getState() {
    return new ElevatorState(io.getPosition());
  }

  /** Stops the elevator motor immediately. */
  public void stop() {
    io.stop();
  }
}
