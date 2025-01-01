package BobcatLib.Subsystems.Swerve.SimpleSwerve.Commands;

import BobcatLib.Hardware.Controllers.Axis;
import BobcatLib.Hardware.Controllers.parser.ControllerJson;
import BobcatLib.Subsystems.Swerve.SimpleSwerve.SwerveDrive;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

/**
 * Command class for teleoperating a swerve drive system using input from suppliers. This class
 * periodically calculates and applies control values for translation, strafing, and rotation, with
 * optional robot-centric or field-centric control.
 */
public class TeleopSwerve extends Command {

  /** Swerve drive subsystem. */
  private SwerveDrive s_Swerve;

  /** Supplier for translation control. */
  private DoubleSupplier translationSup;

  /** Supplier for strafing control. */
  private DoubleSupplier strafeSup;

  /** Supplier for rotation control. */
  private DoubleSupplier rotationSup;

  /** Supplier for enabling robot-centric control. */
  private BooleanSupplier robotCentricSup;

  /** Configuration details parsed from a JSON controller configuration. */
  private ControllerJson controllerJson;

  /**
   * Constructor for the TeleopSwerve class.
   *
   * @param s_Swerve the swerve drive subsystem to control
   * @param translationSup supplier for translation (forward/backward) control
   * @param strafeSup supplier for strafing (side-to-side) control
   * @param rotationSup supplier for rotational control
   * @param robotCentricSup supplier indicating if the drive should be robot-centric
   * @param controllerJson parsed configuration details for driver input and limits
   */
  public TeleopSwerve(
      SwerveDrive s_Swerve,
      DoubleSupplier translationSup,
      DoubleSupplier strafeSup,
      DoubleSupplier rotationSup,
      BooleanSupplier robotCentricSup,
      ControllerJson controllerJson) {
    this.s_Swerve = s_Swerve;
    addRequirements(s_Swerve);

    this.translationSup = translationSup;
    this.strafeSup = strafeSup;
    this.rotationSup = rotationSup;
    this.robotCentricSup = robotCentricSup;
    this.controllerJson = controllerJson;
  }

  /**
   * Periodic execution method called every 20ms during command execution. Calculates drive
   * parameters based on the input suppliers and applies them to the swerve drive system.
   */
  @Override
  public void execute() {
    // Apply deadband to control inputs
    Axis translation = new Axis(translationSup.getAsDouble(), controllerJson.driver.deadband);
    Axis strafe = new Axis(strafeSup.getAsDouble(), controllerJson.driver.deadband);
    Axis rotation = new Axis(rotationSup.getAsDouble(), controllerJson.driver.deadband);

    // Retrieve drive limits from the swerve configuration
    double maxSpeed = s_Swerve.jsonSwerve.moduleSpeedLimits.maxSpeed;
    double maxAngularVelocity = s_Swerve.jsonSwerve.moduleSpeedLimits.maxAngularVelocity;

    // Execute swerve drive command with calculated parameters
    s_Swerve.drive(
        new Translation2d(translation.getDeadband(), strafe.getDeadband()).times(maxSpeed),
        rotation.getDeadband() * maxAngularVelocity,
        !robotCentricSup.getAsBoolean(), // Whether field-centric mode is active
        true, // Always apply safety mechanisms
        s_Swerve.getHeading(),
        s_Swerve.getPose());
  }

  /**
   * Retrieves the current value of the rotation control stick, with the deadband applied.
   *
   * @return the processed value of the rotation stick
   */
  public double getRotationStick() {
    Axis rotation = new Axis(rotationSup.getAsDouble(), controllerJson.driver.deadband);
    return rotation.getDeadband();
  }
}
