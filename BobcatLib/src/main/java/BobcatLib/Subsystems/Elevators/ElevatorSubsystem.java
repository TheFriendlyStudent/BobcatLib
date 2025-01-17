package BobcatLib.Subsystems.Elevators;

import BobcatLib.Subsystems.Elevators.Modules.BaseElevator;
import BobcatLib.Utilities.SetPointWrapper;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ElevatorSubsystem extends SubsystemBase {
  private BaseElevator elevator;
  public SetPointWrapper setPoints;
  public double lowerLimit = 0;
  public double upperLimit = 50;
  public double currentSetPoint = 0;
  public String name;

  public ElevatorSubsystem(String name, BaseElevator elevator, SetPointWrapper points) {
    this.name = name;
    this.elevator = elevator;
    this.setPoints = points;
  }

  @Override
  public void periodic() {
    elevator.periodic();
  }

  public void setNextPosition() {
    elevator.moveElevatorToNext(setPoints);
  }

  public void setPrevPosition() {
    elevator.moveElevatorToPrevious(setPoints);
  }
}
