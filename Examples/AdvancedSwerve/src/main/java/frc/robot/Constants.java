package frc.robot;



import BobcatLib.Subsystems.Swerve.AdvancedSwerve.StandardDeviations.StandardDeviation;
import BobcatLib.Subsystems.Swerve.AdvancedSwerve.StandardDeviations.SwerveStdDevs;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;


public class Constants {
    public static final Mode currentMode = RobotBase.isSimulation() ? Mode.SIM
            : (RobotBase.isReal() ? Mode.REAL : Mode.REPLAY);
    public static final boolean isTuningMode = !DriverStation.isFMSAttached();

    public static enum Mode {
        /** Running on a real robot. */
        REAL,

        /** Running a physics simulator. */
        SIM,

        /** Replaying from a log file. */
        REPLAY
    }

    public static final double loopPeriodSecs = 0.02; // 50 hz, default loop period

    public static final int[] filterTags = new int[]{};
    public static final SwerveStdDevs stdDevs= new SwerveStdDevs(
        new StandardDeviation(0, 0, 0, 0),
        new StandardDeviation(0, 0, 0, 0));

}
