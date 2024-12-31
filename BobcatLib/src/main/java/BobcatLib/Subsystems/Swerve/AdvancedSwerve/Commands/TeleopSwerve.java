package BobcatLib.Subsystems.Swerve.AdvancedSwerve.Commands;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import BobcatLib.Subsystems.Swerve.AdvancedSwerve.Swerve.SwerveDrive;
import BobcatLib.Subsystems.Swerve.AdvancedSwerve.Swerve.SwerveConstants;
import BobcatLib.Subsystems.Swerve.AdvancedSwerve.Swerve.SwerveConstants.Limits;
import BobcatLib.Subsystems.Swerve.AdvancedSwerve.Swerve.Utility.Assists.RotationalAssist;
import BobcatLib.Subsystems.Swerve.AdvancedSwerve.Swerve.Utility.Assists.TranslationAssist;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;

public class TeleopSwerve extends Command {
    private SwerveDrive swerve;
    private DoubleSupplier translation;
    private DoubleSupplier fineStrafe;
    private DoubleSupplier strafe;
    private DoubleSupplier rotation;
    private BooleanSupplier robotCentric;
    private DoubleSupplier fineTrans;
   
    private TranslationAssist aimAssist;
    private RotationalAssist rotAssist;
    /**
     * 
     * suppliers are objects that can return a value that will change, for example a
     * method that returns a double can be input as a doubleSupplier
     * 
     * @param swerve        the swerve subsystem
     * @param translation   [-1,1] forward and backward
     * @param strafe        [-1,1] value to drive the robot left and right
     * @param rotation      [-1,1] value to rotate the drivetrain
     * @param robotCentric  field-centric if false
     * @param fineStrafe    [-1,1] slow speed control, cancled if translation or strafe is in use
     * @param fineTrans     [-1,1] slow speed control, cancled if translation or strafe is in use
     * @param translationAssist     Guides the chassis towards the supplied 
     * @param autoAlignSupplier should we automatically align to the speaker
     */
    public TeleopSwerve(SwerveDrive swerve, DoubleSupplier translation, DoubleSupplier strafe,
            DoubleSupplier rotation, BooleanSupplier robotCentric, DoubleSupplier fineStrafe, DoubleSupplier fineTrans,
            TranslationAssist translationAssist, RotationalAssist rotationalAssist) {
        
        aimAssist = translationAssist;
        rotAssist = rotationalAssist;
        
        this.swerve = swerve;
        addRequirements(swerve);

        this.translation = translation;
        this.strafe = strafe;
        this.rotation = rotation;
        this.robotCentric = robotCentric;
        this.fineStrafe = fineStrafe;
        this.fineTrans = fineTrans;
        }

    @Override
    public void execute() {

        /* Get Values, Deadband */
        double translationVal = MathUtil.applyDeadband(translation.getAsDouble(), SwerveConstants.stickDeadband);
        double strafeVal = MathUtil.applyDeadband(strafe.getAsDouble(), SwerveConstants.stickDeadband);
        double rotationVal = MathUtil.applyDeadband(rotation.getAsDouble(), SwerveConstants.stickDeadband); // from 0 to one

        /*
         * If joysticks not receiving any normal input, use twist values for fine adjust
         */
        if (strafeVal == 0.0) {
            strafeVal = fineStrafe.getAsDouble();
        }
        if (translationVal == 0.0) {
            translationVal = fineTrans.getAsDouble();
        }
        

      

        /* Drive */
        swerve.drive(
                new Translation2d(translationVal, strafeVal).times(Limits.Chassis.maxSpeed),
                rotationVal * Limits.Chassis.maxAngularVelocity.getRadians(),
                !robotCentric.getAsBoolean(),
                aimAssist,
                rotAssist);

    }
}