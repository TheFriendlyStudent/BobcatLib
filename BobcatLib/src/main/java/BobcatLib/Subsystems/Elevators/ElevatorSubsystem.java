package BobcatLib.Subsystems.Elevators;

import BobcatLib.Subsystems.Elevators.Modules.BaseElevator;
import BobcatLib.Subsystems.Elevators.Modules.ElevatorIO;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * Represents the elevator subsystem, responsible for controlling and configuring the elevator
 * hardware and logic. It integrates with an {@link ElevatorIO} implementation for hardware
 * abstraction.
 */
public class ElevatorSubsystem extends SubsystemBase {
  private final BaseElevator elevatorModule;
  private String name = "";

  /**
   * Constructs a new {@code ElevatorSubsystem}.
   *
   * @param name The name of the subsystem, used for identification or debugging.
   * @param elevatorModule The {@link BaseElevator} implementation for controlling the elevator
   *     hardware.
   */
  public ElevatorSubsystem(String name, BaseElevator elevatorModule) {
    this.name = name;
    this.elevatorModule = elevatorModule;
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

  /** Stops the elevator motor immediately. */
  public void stop() {
    elevatorModule.stop();
  }
}
