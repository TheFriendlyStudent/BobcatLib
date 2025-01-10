package frc.robot;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ElevatorSubsystem extends SubsystemBase {
    public FalconElevatorMotor motor = new FalconElevatorMotor(2,"CANt_open_file");
    public SetPointWrapper setPoints = new SetPointWrapper("0,10,20,30,40,50");

    
    @Override
    public void periodic() {
      SmartDashboard.putNumber("Elevator Position", motor.getPosition());
      
    }
  /**
   * Sets the elevator motor output to a specified percentage.
   *
   * @param percent The desired output percentage, where 1.0 represents full forward power and -1.0
   *     represents full reverse power.
   */
  public void setPercentOut(double percent) {
    motor.setControl(percent);
  }

  /**
   * Moves the elevator to the specified position.
   *
   * @param position The desired position of the elevator as a {@link Rotation2d} object.
   */
  public void moveElevator(Rotation2d position) {
    motor.setAngle(position.getRotations());
  }

  public void moveElevatorToNext() {
    Rotation2d currentPosition = Rotation2d.fromRotations(motor.getPosition());
    Rotation2d nextPosition =
        Rotation2d.fromRotations(setPoints.getSurroundingPoints(currentPosition).get(1));
    moveElevator(nextPosition);
  }

  public void holdPosition() {
    Rotation2d currentPosition = Rotation2d.fromRotations(motor.getPosition());
    moveElevator(currentPosition);
  }

  /** Stops the elevator motor immediately. */
  public void stop() {
    motor.stopMotor();
  }
}
