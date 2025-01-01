package BobcatLib.Subsystems.Climbers;

import BobcatLib.Subsystems.Climbers.Modules.ClimberIO;
import BobcatLib.Subsystems.Climbers.Modules.ClimberIO.ClimberIOInputs;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

/**
 * Represents the climber subsystem, responsible for managing the climber's behavior and
 * periodically updating its inputs.
 */
public class ClimberSubsystem extends SubsystemBase {

  /** The logged inputs for the climber, used for telemetry and diagnostics. */
  private final ClimberIOInputs inputs = new ClimberIOInputs();

  /** The climber's input-output interface for hardware interaction. */
  private ClimberIO io;

  /** The current motor output percentage (-1.0 to 1.0). */
  private double percent = 0;

  /** The position being held by the climber mechanism, in rotations. */
  private double holdingPos = 0;

  /** The name of the climber subsystem instance. */
  private String name = "";

  /**
   * Constructs a {@code ClimberSubsystem} with the specified name and input-output interface.
   *
   * @param name The name of the climber subsystem.
   * @param io The {@link ClimberIO} interface for hardware interaction.
   */
  public ClimberSubsystem(String name, ClimberIO io) {
    this.name = name;
    this.io = io;
    holdingPos = inputs.climberMotorPosition;
  }

  /**
   * Periodically updates the climber's inputs and handles its control behavior.
   *
   * <p>This method is called regularly by the scheduler to update telemetry and control logic. It
   * also uses AdvantageKit's {@link Logger} to log the climber's inputs for diagnostics.
   */
  @Override
  public void periodic() {
    io.updateInputs(inputs);
    if (percent == 0) {
      io.holdPos(holdingPos);
    } else {
      holdingPos = inputs.climberMotorPosition;
    }
  }

  /**
   * Sets the climber motor's output percentage.
   *
   * <p>The output value should be between -1.0 (full reverse) and 1.0 (full forward).
   *
   * @param percent The desired motor output percentage (-1.0 to 1.0).
   */
  public void setPercentOut(double percent) {
    io.setPercentOut(percent);
    this.percent = percent;
  }

  /**
   * Stops the climber mechanism by setting motor output to zero.
   *
   * <p>This method also resets the output percentage tracking.
   */
  public void stop() {
    io.stop();
    percent = 0;
  }
}
