package BobcatLib.Subsystems.Swerve.AdvancedSwerve;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

public class TeleopSwerve extends Command {
  private SwerveBase swerve;
  private DoubleSupplier translation;
  private DoubleSupplier fineStrafe;
  private DoubleSupplier strafe;
  private DoubleSupplier rotation;
  private BooleanSupplier robotCentric;
  private DoubleSupplier fineTrans;
  private BooleanSupplier autoAlignSupplier;
  private BooleanSupplier aimAssistSupplier;
  private PIDController aimAssistXController = new PIDController(0.2, 0, 0); // TODO tune
  private PIDController aimAssistYController = new PIDController(0.2, 0, 0);
  private Translation2d currTranslation = new Translation2d();
  private double stickDeadband;
  private double maxVelocity;
  private Rotation2d maxAngularVelocity;

  /**
   * creates a command to control your swerve in teleop. Suppliers are methods that you pass in, for
   * example, a BooleanSupplier is a method that returns a boolean when called
   *
   * @param swerve your swerve subsystem
   * @param translation forward-back [-1,1]
   * @param strafe left-right [-1,1]
   * @param rotation ccw+ [-1,1]
   * @param robotCentric when true, forward will be relative to the front of the robot, not the
   *     field
   * @param fineStrafe [-1,1]
   * @param fineTrans [-1,1]
   * @param aimAssistSupplier when true, aim assist will be active
   * @param autoAlignSupplier when true, autoalign will be active
   * @param stickDeadband stick values less than this will be rounded to 0, useful for sticks with
   *     drift
   */
  public TeleopSwerve(
      SwerveBase swerve,
      DoubleSupplier translation,
      DoubleSupplier strafe,
      DoubleSupplier rotation,
      BooleanSupplier robotCentric,
      DoubleSupplier fineStrafe,
      DoubleSupplier fineTrans,
      BooleanSupplier aimAssistSupplier,
      BooleanSupplier autoAlignSupplier,
      double stickDeadband,
      double maxChassisSpeedMetersPerSecond,
      Rotation2d maxChassisAngularVelocity) {

    this.swerve = swerve;
    addRequirements(swerve);

    this.translation = translation;
    this.strafe = strafe;
    this.rotation = rotation;
    this.robotCentric = robotCentric;
    this.fineStrafe = fineStrafe;
    this.fineTrans = fineTrans;
    this.aimAssistSupplier = aimAssistSupplier;
    this.autoAlignSupplier = autoAlignSupplier;
    this.stickDeadband = stickDeadband;
    maxVelocity = maxChassisSpeedMetersPerSecond;
    maxAngularVelocity = maxChassisAngularVelocity;
  }

  @Override
  public void execute() {

    /* Get Values, Deadband */
    double translationVal = MathUtil.applyDeadband(translation.getAsDouble(), stickDeadband);
    double strafeVal = MathUtil.applyDeadband(strafe.getAsDouble(), stickDeadband);
    double rotationVal =
        MathUtil.applyDeadband(rotation.getAsDouble(), stickDeadband); // from 0 to one

    /*
     * If joysticks not receiving any normal input, use twist values for fine adjust
     */
    if (strafeVal == 0.0) {
      strafeVal = fineStrafe.getAsDouble();
    }
    if (translationVal == 0.0) {
      translationVal = fineTrans.getAsDouble();
    }

    swerve.setAimAssistTranslation(new Translation2d());
    swerve.setAutoAlignAngle(new Rotation2d());

    if (aimAssistSupplier.getAsBoolean()) {
      currTranslation = swerve.getPose().getTranslation();
      translationVal += aimAssistXController.calculate(currTranslation.getX());
      strafeVal += aimAssistYController.calculate(currTranslation.getY());
    }

    /* Drive */
    swerve.drive(
        new Translation2d(translationVal, strafeVal).times(maxVelocity),
        rotationVal * maxAngularVelocity.getRadians(),
        !robotCentric.getAsBoolean(),
        autoAlignSupplier.getAsBoolean());
  }
}
