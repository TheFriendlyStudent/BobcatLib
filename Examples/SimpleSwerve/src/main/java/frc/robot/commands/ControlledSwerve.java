package frc.robot.commands;

import BobcatLib.Hardware.Controllers.Axis;
import BobcatLib.Hardware.Controllers.parser.ControllerJson;
import BobcatLib.Subsystems.Swerve.SimpleSwerve.SwerveDrive;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;

/** Controller Swerve */
public class ControlledSwerve extends Command {
  private SwerveDrive s_Swerve;
  private Double translationSup;
  private Double strafeSup;
  private Double rotationSup;
  private boolean robotCentricSup;
  private ControllerJson controllerJson;
  /**
   * Controller Swerver
   *
   * @param s_Swerve Swerve Drive
   * @param translationSup Translation Suplier
   * @param strafeSup Strafe Suplier
   * @param rotationSup Rotation Supplier
   * @param robotCentricSup Robot Centric Supplier
   */
  public ControlledSwerve(
      SwerveDrive s_Swerve,
      Double translationSup,
      Double strafeSup,
      Double rotationSup,
      boolean robotCentricSup,
      ControllerJson controllerJson) {
    this.s_Swerve = s_Swerve;
    addRequirements(s_Swerve);

    this.translationSup = translationSup;
    this.strafeSup = strafeSup;
    this.rotationSup = rotationSup;
    this.robotCentricSup = robotCentricSup;
    this.controllerJson = controllerJson;
  }

  /** Executes every 20ms */
  @Override
  public void execute() {
    Axis translation = new Axis(translationSup, controllerJson.driver.deadband);
    Axis strafe = new Axis(strafeSup, controllerJson.driver.deadband);
    Axis rotation = new Axis(rotationSup, controllerJson.driver.deadband);
    // Limits
    double maxSpeed = s_Swerve.jsonSwerve.moduleSpeedLimits.maxSpeed;
    double maxAngularVelocity = s_Swerve.jsonSwerve.moduleSpeedLimits.maxAngularVelocity;
    /* Drive */
    s_Swerve.drive(
        new Translation2d(translation.getDeadband(), strafe.getDeadband()).times(maxSpeed),
        rotation.getDeadband() * maxAngularVelocity,
        !robotCentricSup,
        true, s_Swerve.getHeading(), s_Swerve.getPose());
  }
}
