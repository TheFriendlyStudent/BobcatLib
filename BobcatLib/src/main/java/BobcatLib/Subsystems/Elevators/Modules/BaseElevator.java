package BobcatLib.Subsystems.Elevators.Modules;

import BobcatLib.Subsystems.Elevators.Modules.ElevatorIO.ElevatorIOInputs;
import BobcatLib.Subsystems.Elevators.Utility.Parser.ElevatorJson;
import com.fasterxml.jackson.databind.ObjectMapper;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Filesystem;
import java.io.File;
import java.io.IOException;

public class BaseElevator {
  private ElevatorIO io;
  private ElevatorIOInputs inputs = new ElevatorIOInputs();
  private ElevatorJson elevatorJson;

  public BaseElevator(ElevatorIO io) {
    this.io = io;
  }

  public void periodic() {
    io.updateInputs(inputs);
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

    // Load the configuration into the IO interface
    io.loadConfigurationFromFile(elevatorJson);
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

  /** Stops the elevator motor immediately. */
  public void stop() {
    io.stop();
  }
}
