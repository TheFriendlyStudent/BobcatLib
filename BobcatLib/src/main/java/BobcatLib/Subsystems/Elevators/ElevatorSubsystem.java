package BobcatLib.Subsystems.Elevators;

import BobcatLib.Hardware.Motors.BaseMotor;
import BobcatLib.Hardware.Motors.MotorConfigs;
import BobcatLib.Hardware.Motors.SensorHelpers.NeutralModeWrapper.disabledMode;
import BobcatLib.Hardware.Motors.SensorHelpers.SensorDirectionWrapper;
import BobcatLib.Hardware.Motors.Utility.SoftwareLimitWrapper;
import BobcatLib.Hardware.Motors.Utility.SoftwareLimitWrapper.SoftwareLimitType;
import BobcatLib.Subsystems.Elevators.Modules.BaseElevator;
import BobcatLib.Subsystems.Elevators.Modules.ElevatorIO;
import BobcatLib.Subsystems.Elevators.Modules.ElevatorReal;
import BobcatLib.Subsystems.Elevators.Utility.Parser.ElevatorJson;
import BobcatLib.Utilities.CANDeviceDetails;
import BobcatLib.Utilities.CANDeviceDetails.Manufacturer;
import BobcatLib.Utilities.SetPointWrapper;
import com.fasterxml.jackson.databind.ObjectMapper;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.io.File;
import java.io.IOException;

/**
 * Represents the elevator subsystem, responsible for controlling and configuring the elevator
 * hardware and logic. It integrates with an {@link ElevatorIO} implementation for hardware
 * abstraction.
 */
public class ElevatorSubsystem extends SubsystemBase {
  private final BaseElevator elevatorModule;
  private String name = "";
  private ElevatorJson elevatorJson;
  /**
   * Constructs a new {@code ElevatorSubsystem}.
   *
   * @param name The name of the subsystem, used for identification or debugging. hardware.
   */
  public ElevatorSubsystem(String name) {
    this.name = name;
    loadConfigurationFromFile();

    // THe following code is NASTY , please refactor and clean up.
    CANDeviceDetails details =
        new CANDeviceDetails(elevatorJson.motor.id, elevatorJson.motor.canbus, Manufacturer.Ctre);
    SoftwareLimitWrapper slw =
        new SoftwareLimitWrapper(
            elevatorJson.limits.lowerLimits,
            elevatorJson.limits.upperLimits,
            SoftwareLimitType.BOTH);
    MotorConfigs mc = new MotorConfigs();
    mc.canDevice = details;
    mc.kD = elevatorJson.elevatorPID.driveKD;
    mc.kI = elevatorJson.elevatorPID.driveKI;
    mc.kP = elevatorJson.elevatorPID.driveKP;
    mc.isInverted = elevatorJson.motor.inverted;
    mc.mode = disabledMode.Brake;
    mc.motorToGearRatio = 1;
    mc.sensorDirection = new SensorDirectionWrapper(mc.isInverted);
    this.elevatorModule =
        new BaseElevator(new ElevatorReal(loadMotor(mc, slw, elevatorJson.motor.motor_type)));
  }

  public BaseMotor loadMotor(MotorConfigs mc, SoftwareLimitWrapper slw, String motor_type) {
    return new BaseMotor(mc.canDevice, mc, motor_type, slw);
  }

  /**
   * Loads the elevator configuration from a JSON file located in the robot's deploy directory. The
   * file should be in the "configs/Elevator" folder and named "elevator.json".
   */
  public void loadConfigurationFromFile() {
    // Get the deploy directory where the configuration file is located
    File deployDirectory = Filesystem.getDeployDirectory();
    assert deployDirectory.exists();

    // Define the directory path and the JSON configuration file path
    File directory = new File(deployDirectory, "configs/Elevator");
    assert new File(directory, "elevator.json").exists();
    File intakeFile = new File(directory, "elevator.json");
    assert intakeFile.exists();

    // Parse the configuration JSON file
    elevatorJson = new ElevatorJson();
    try {
      elevatorJson = new ObjectMapper().readValue(intakeFile, ElevatorJson.class);
    } catch (IOException e) {
      // Handle the case where reading the file fails (logging or exception handling
      // may be added
      // here)
    }
  }

  /** Periodically updates the elevator subsystem inputs. Called once per scheduler run. */
  @Override
  public void periodic() {
    elevatorModule.periodic();
  }

  /**
   * Sets the elevator motor output to a specified percentage.
   *
   * @param percent The desired output percentage, where 1.0 represents full forward power and -1.0
   *     represents full reverse power.
   */
  public void setPercentOut(double percent) {
    elevatorModule.setPercentOut(percent);
  }

  /**
   * Moves the elevator to the specified position.
   *
   * @param position The desired position of the elevator as a {@link Rotation2d} object.
   */
  public void moveElevator(Rotation2d position) {
    elevatorModule.moveElevator(position);
  }

  public void moveElevatorToNext() {
    Rotation2d currentPosition = elevatorModule.getState().getElevatorRotations();
    Rotation2d nextPosition =
        Rotation2d.fromRotations(
            new SetPointWrapper("0,10,20,30,40,50").getSurroundingPoints(currentPosition).get(1));
    elevatorModule.moveElevator(nextPosition);
  }

  public void holdPosition() {
    Rotation2d current = elevatorModule.getState().getElevatorRotations();
    elevatorModule.moveElevator(current);
  }

  /** Stops the elevator motor immediately. */
  public void stop() {
    elevatorModule.stop();
  }
}
