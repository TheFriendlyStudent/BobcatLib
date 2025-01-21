package BobcatLib.Subsystems.Vision;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * Represents the Intake Subsystem in the robot. It interacts with the intake module to control the
 * intake mechanism, load configurations from a file, and periodically update inputs.
 */
public class VisionSubsystem extends SubsystemBase {
  public final String name;
  /**
   * Constructor for the VisionSubsystem.
   *
   * @param name The name of the Vision subsystem.
   */
  public VisionSubsystem(String name) {
    this.name = name;
  }

  /**
   * Periodically updates the input values for the intake subsystem and logs them. This method is
   * typically called every robot cycle.
   */
  public void periodic() {}
}
