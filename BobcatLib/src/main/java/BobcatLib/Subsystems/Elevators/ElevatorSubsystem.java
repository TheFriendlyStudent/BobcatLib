package BobcatLib.Subsystems.Elevators;

import BobcatLib.Subsystems.Elevators.Modules.ElevatorModuleIO;
import BobcatLib.Utilities.SetPointWrapper;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ElevatorSubsystem extends SubsystemBase {
  private ElevatorModuleIO io;
  public SetPointWrapper setPoints;
  public double lowerLimit = 0;
  public double upperLimit = 50;
  public double currentSetPoint = 0;
  public String name;

  public ElevatorSubsystem(String name, ElevatorModuleIO io) {
    this.io = io;
    this.name = name;
  }

  @Override
  public void periodic() {

  }

  /**
   * Moves the elevator to the specified position.
   *
   * @param position The desired position of the elevator as a {@link Rotation2d}
   *                 object.
   */
  public void moveElevator(Rotation2d position) {
    io.moveElevator(position);
  }

  public void moveElevatorToNext() {
    io.moveElevatorToNext();
  }

  public void moveElevatorToPrevious() {
    io.moveElevatorToPrevious();
  }

  public void holdPosition() {
    io.holdPosition();
  }

  /** Stops the elevator motor immediately. */
  public void stop() {
    io.stop();
  }
}
