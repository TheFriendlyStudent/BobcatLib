package BobcatLib.Subsystems.Swerve.SimpleSwerve;

import BobcatLib.Hardware.Controllers.OI;
import BobcatLib.Subsystems.Swerve.SimpleSwerve.Commands.ControlledSwerve;
import BobcatLib.Subsystems.Swerve.SimpleSwerve.Commands.TeleopSwerve;
import BobcatLib.Subsystems.Swerve.SimpleSwerve.Constants.SwerveConstants;
import BobcatLib.Subsystems.Swerve.SimpleSwerve.Utility.Alliance;
import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the robot periodic
 * methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class SwerveContainer {
  /* Subsystems */
  public final OI s_Controls = new OI();
  public final SwerveDrive s_Swerve;
  private final LoggedDashboardChooser<Command> autoChooser =
      new LoggedDashboardChooser<>("Auto Routine");
  public Alliance alliance;
  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public SwerveContainer(boolean isSimulation, DriverStation.Alliance ally) {
    alliance = new Alliance(ally);
    s_Swerve = new SwerveDrive(isSimulation, alliance, SwerveConstants.visionStdDevs, SwerveConstants.stateStdDevs);
    initComand();
    // Auto controls
    /*
     * Auto Chooser
     *
     * Names must match what is in PathPlanner
     * Please give descriptive names
     */
    autoChooser.addDefaultOption("Do Nothing", Commands.none());
    // autoChooser.addOption("name", new PathPlannerAuto("nameinpathplanner"));
    // Configure the button bindings
    configureButtonBindings();
  }

  public void initComand() {
    DoubleSupplier translation = () -> s_Controls.getTranslationValue(); //TODO change if 2025 field isn't mirrored
    DoubleSupplier strafe = () -> s_Controls.getStrafeValue();
    if (!alliance.isBlueAlliance()) {
      translation = () -> -s_Controls.getTranslationValue();
      strafe = () -> -s_Controls.getStrafeValue();
    }
    s_Swerve.setDefaultCommand(
        new TeleopSwerve(
            s_Swerve,
            translation,
            strafe,
            () -> s_Controls.getRotationValue(),
            () -> s_Controls.robotCentric.getAsBoolean(),
            s_Controls.controllerJson));
  }

  public boolean autoChooserInitialized() {
    return autoChooser.get() != null;
  }

  /** this should only be called once DS and FMS are attached */
  public void configureAutos() {

    /*
     * Auto Events
     *
     * Names must match what is in PathPlanner
     * Please give descriptive names
     */
    NamedCommands.registerCommand("PathfindingCommand", s_Swerve.driveToPose(new Pose2d()));

    /*
     * Auto Chooser
     *
     * Names must match what is in PathPlanner
     * Please give descriptive names
     */
    autoChooser.addDefaultOption("Do Nothing", Commands.none());
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    Command zeroGyro = Commands.runOnce(s_Swerve::zeroHeading);
    /* Driver Buttons */
    s_Controls.zeroGyro.onTrue(zeroGyro);
    // Cardinal Modes
    double maxSpeed = s_Swerve.jsonSwerve.chassisSpeedLimits.maxSpeed;
    Command strafeBack =
        s_Swerve.driveAsCommand(new Translation2d(-1, 0).times(maxSpeed)).repeatedly();
    Command strafeForward =
        s_Swerve.driveAsCommand(new Translation2d(1, 0).times(maxSpeed)).repeatedly();
    Command strafeLeft =
        s_Swerve.driveAsCommand(new Translation2d(0, 1).times(maxSpeed)).repeatedly();
    Command strafeRight =
        s_Swerve.driveAsCommand(new Translation2d(0, -1).times(maxSpeed)).repeatedly();
    s_Controls.dpadForwardBtn.whileTrue(strafeForward);
    s_Controls.dpadBackBtn.whileTrue(strafeBack);
    s_Controls.dpadRightBtn.whileTrue(strafeRight);
    s_Controls.dpadLeftBtn.whileTrue(strafeLeft);
  }

  /**
   * Use this to pass the autonomous command to the main class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return autoChooser.get();
  }

  /**
   * Use this to pass the test command to the main class. Control pattern is forward, right ,
   * backwards, left, rotate in place clockwise, rotate in place counterclowise, forward while
   * rotating Clockwise, forward while rotating counter clockwise
   *
   * @return the command to run in autonomous
   */
  public Command getTestCommand() {
    Command testSwerveForward =
        new ControlledSwerve(s_Swerve, 0.2, 0.0, 0.0, false, s_Controls.controllerJson)
            .withTimeout(3);
    Command testSwerveRight =
        new ControlledSwerve(s_Swerve, 0.0, 0.2, 0.0, false, s_Controls.controllerJson)
            .withTimeout(3);
    Command testSwerveBackwards =
        new ControlledSwerve(s_Swerve, -0.2, 0.0, 0.0, false, s_Controls.controllerJson)
            .withTimeout(3);
    Command testSwerveLeft =
        new ControlledSwerve(s_Swerve, 0.0, -0.2, 0.0, false, s_Controls.controllerJson)
            .withTimeout(3);
    Command testRIPCW =
        new ControlledSwerve(s_Swerve, 0.0, 0.0, 0.2, false, s_Controls.controllerJson)
            .withTimeout(3);
    Command testRIPCCW =
        new ControlledSwerve(s_Swerve, 0.0, 0.0, -0.2, false, s_Controls.controllerJson)
            .withTimeout(3);
    Command stopMotorsCmd = new InstantCommand(() -> s_Swerve.stopMotors());
    Command testCommand =
        testSwerveForward
            .andThen(testSwerveRight)
            .andThen(testSwerveBackwards)
            .andThen(testSwerveLeft)
            .andThen(testRIPCW)
            .andThen(testRIPCCW)
            .andThen(stopMotorsCmd);
    return testCommand;
  }
}
