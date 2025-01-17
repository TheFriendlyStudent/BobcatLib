package frc.robot;

import java.io.File;
import java.io.IOException;

import com.fasterxml.jackson.databind.ObjectMapper;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Parser.ElevatorModuleJson;

public class ElevatorSubsystem extends SubsystemBase {
    public FalconElevatorMotor motor;
    public SetPointWrapper setPoints = new SetPointWrapper("0,10,20,30,40,50");
    public double lowerLimit = 0;
    public double upperLimit = 50;
    public double currentSetPoint = 0;
    public ElevatorModuleJson elevatorJson;
    public ElevatorSubsystem(){
      loadConfigurationFromFile();
      motor = new FalconElevatorMotor(9,"");
      motor.setPosition(0);
    }

      /**
   * Loads the configuration for the intake subsystem from a JSON file located in the robot's deploy
   * directory. The file is expected to be in the "configs/Intake" folder.
   */
  public void loadConfigurationFromFile() {
    // Get the deploy directory where the configuration file is located
    File deployDirectory = Filesystem.getDeployDirectory();
    assert deployDirectory.exists();

    // Define the directory path and the intake JSON file path
    File directory = new File(deployDirectory, "configs/Elevator");
    assert new File(directory, "elevator.json").exists();
    File intakeFile = new File(directory, "elevator.json");
    assert intakeFile.exists();

    // Parse the intake configuration JSON file
    elevatorJson = new ElevatorModuleJson();
    try {
      elevatorJson = new ObjectMapper().readValue(intakeFile, ElevatorModuleJson.class);
    } catch (IOException e) {
      // Handle the case where reading the file fails (you may want to log or throw an exception)
    }
  }
    @Override
    public void periodic() {
      SmartDashboard.putNumber("Elevator Position", motor.getPosition());
      SmartDashboard.putNumber("Elevator PID Error", motor.getErrorPosition());
      
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
    if( currentPosition.getRotations() > currentSetPoint ){
      holdPosition();
    }
    Rotation2d nextPosition =
        Rotation2d.fromRotations(setPoints.getSurroundingPoints(currentPosition).get(1));
    currentSetPoint = nextPosition.getRotations();
    moveElevator(nextPosition);
  }
  public void moveElevatorToPrevious() {
    Rotation2d currentPosition = Rotation2d.fromRotations(motor.getPosition());
    if( currentPosition.getRotations() < currentSetPoint ){
      holdPosition();
    }
    Rotation2d previousPosition =
        Rotation2d.fromRotations(setPoints.getSurroundingPoints(currentPosition).get(0));
    currentSetPoint = previousPosition.getRotations();
    moveElevator(previousPosition);
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
