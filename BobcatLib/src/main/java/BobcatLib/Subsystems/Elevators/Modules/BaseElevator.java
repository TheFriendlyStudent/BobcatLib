package BobcatLib.Subsystems.Elevators.Modules;

import BobcatLib.Subsystems.Elevators.Modules.ElevatorModuleIO.ElevatorIOInputs;
import BobcatLib.Utilities.SetPointWrapper;
import edu.wpi.first.math.geometry.Rotation2d;

public class BaseElevator {
  private final ElevatorModuleIO io;
  private final ElevatorIOInputs inputs = new ElevatorIOInputs();
  private Rotation2d currentSetPoint = new Rotation2d();

  public BaseElevator(ElevatorModuleIO io) {
    this.io = io;
  }

  public void periodic() {
    Rotation2d currentSetPoint = getCurrentSetPoint();
    io.updateInputs(inputs, currentSetPoint.getRotations());
    if (io.getPosition() == getCurrentSetPoint()) {
      holdPosition();
    } else {
      io.moveElevator(currentSetPoint);
    }
  }

  public void moveElevatorToNext(SetPointWrapper setPoints) {
    Rotation2d currentPosition = Rotation2d.fromRotations(io.getPosition().getRotations());
    Rotation2d nextPosition =
        Rotation2d.fromRotations(setPoints.getSurroundingPoints(currentPosition).get(1));
    currentSetPoint = nextPosition;
  }

  public void moveElevatorToPrevious(SetPointWrapper setPoints) {
    Rotation2d currentPosition = Rotation2d.fromRotations(io.getPosition().getRotations());
    Rotation2d previousPosition =
        Rotation2d.fromRotations(setPoints.getSurroundingPoints(currentPosition).get(0));
    currentSetPoint = previousPosition;
  }

  public void holdPosition() {
    Rotation2d currentPosition = Rotation2d.fromRotations(io.getPosition().getRotations());
    currentSetPoint = currentPosition;
  }

  public Rotation2d getCurrentSetPoint() {
    return currentSetPoint;
  }

  /** Stops the elevators motor immediately. */
  public void stop() {
    io.stop();
  }
}
