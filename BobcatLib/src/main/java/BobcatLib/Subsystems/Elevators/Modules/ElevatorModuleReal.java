package BobcatLib.Subsystems.Elevators.Modules;

import BobcatLib.Subsystems.Elevators.Modules.Motors.BaseElevatorMotor;
import edu.wpi.first.math.geometry.Rotation2d;

public class ElevatorModuleReal implements ElevatorModuleIO {
  private BaseElevatorMotor motor;

  public ElevatorModuleReal() {}

  public void updateInputs(ElevatorIOInputs inputs, double currentSetPoint) {
    inputs.elevatorPosition = getPosition().getRotations();
    inputs.currentSetPoint = currentSetPoint;
  }

  public void moveElevator(Rotation2d setPoint) {
    motor.setAngle(setPoint.getRotations());
  }

  public Rotation2d getPosition() {
    return Rotation2d.fromRotations(motor.getPosition());
  }

  /** Stops the elevator motor immediately. */
  public void stop() {
    motor.stopMotor();
  }
}
