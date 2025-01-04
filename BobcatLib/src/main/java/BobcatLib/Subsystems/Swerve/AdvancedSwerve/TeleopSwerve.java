package BobcatLib.Subsystems.Swerve.AdvancedSwerve;

import BobcatLib.Subsystems.Swerve.AdvancedSwerve.Assists.AimAssist;
import BobcatLib.Subsystems.Swerve.AdvancedSwerve.Assists.AutoAlign;
import BobcatLib.Subsystems.Swerve.AdvancedSwerve.Constants.SwerveConstants;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.Logger;

public class TeleopSwerve extends Command {
  private SwerveBase swerve;
  private DoubleSupplier translation;
  private DoubleSupplier fineStrafe;
  private DoubleSupplier strafe;
  private DoubleSupplier rotation;
  private BooleanSupplier robotCentric;
  private DoubleSupplier fineTrans;
  private AutoAlign autoAlign;
  private AimAssist aimAssist;
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
      AimAssist aimAssist,
      AutoAlign autoAlign,
      double stickDeadband,
      SwerveConstants constants) {

    this.swerve = swerve;
    addRequirements(swerve);

    this.translation = translation;
    this.strafe = strafe;
    this.rotation = rotation;
    this.robotCentric = robotCentric;
    this.fineStrafe = fineStrafe;
    this.fineTrans = fineTrans;
    this.aimAssist = aimAssist;
    this.autoAlign = autoAlign;
    this.stickDeadband = stickDeadband;
    maxVelocity = constants.speedLimits.chassisLimits.maxVelocity;
    maxAngularVelocity = constants.speedLimits.chassisLimits.maxAngularVelocity;
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

    // if aim assist is active, update translation and strafe values
    // to nudge robot towards desired pose on top of driver input
    if (aimAssist.active()) {
      currTranslation = swerve.getPose().getTranslation();

      translationVal += aimAssist.outputX(currTranslation);
      strafeVal += aimAssist.outputY(currTranslation);

      Logger.recordOutput("Swerve/Assists/AimAssistOutputX", aimAssist.outputX(currTranslation));
      Logger.recordOutput("Swerve/Assists/AimAssistOutputY", aimAssist.outputY(currTranslation));
    }
    Logger.recordOutput("Swerve/Assists/AimAssistActive", aimAssist.active());

    Logger.recordOutput("Swerve/DesiredTranslation", translationVal);
    Logger.recordOutput("Swerve/DesiredStrafe", strafeVal);

    /* Drive */
    swerve.drive(
        new Translation2d(translationVal, strafeVal).times(maxVelocity),
        rotationVal * maxAngularVelocity.getRadians(),
        !robotCentric.getAsBoolean());
  }
}
