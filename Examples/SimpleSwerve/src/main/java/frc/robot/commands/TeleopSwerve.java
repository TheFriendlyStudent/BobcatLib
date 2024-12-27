package frc.robot.commands;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import BobcatLib.Hardware.Controllers.Axis;
import BobcatLib.Hardware.Controllers.parser.ControllerJson;
import BobcatLib.Subsystems.Swerve.SimpleSwerve.SwerveDrive;

public class TeleopSwerve extends Command {
  private SwerveDrive s_Swerve;
  private DoubleSupplier translationSup;
  private DoubleSupplier strafeSup;
  private DoubleSupplier rotationSup;
  private BooleanSupplier robotCentricSup;
  private ControllerJson controllerJson;

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

  @Override
  public void execute() {
    Axis translation = new Axis(translationSup.getAsDouble(), controllerJson.driver.deadband);
    Axis strafe = new Axis(strafeSup.getAsDouble(), controllerJson.driver.deadband);
    Axis rotation = new Axis(rotationSup.getAsDouble(), controllerJson.driver.deadband);
    // Limits
    double maxSpeed = s_Swerve.jsonSwerve.moduleSpeedLimits.maxSpeed;
    double maxAngularVelocity = s_Swerve.jsonSwerve.moduleSpeedLimits.maxAngularVelocity;
    /* Drive */
    s_Swerve.drive(
        new Translation2d(translation.getDeadband(), strafe.getDeadband()).times(maxSpeed),
        rotation.getDeadband() * maxAngularVelocity,
        !robotCentricSup.getAsBoolean(),
        true, s_Swerve.getHeading(), s_Swerve.getPose());
  }

  public double getRotationStick() {
    Axis rotation = new Axis(rotationSup.getAsDouble(), controllerJson.driver.deadband);
    return rotation.getDeadband();
  }
}
