package BobcatLib.Subsystems.Swerve.AdvancedSwerve;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;


public class Constants {
    public static final Mode currentMode = RobotBase.isSimulation() ? Mode.SIM
            : (RobotBase.isReal() ? Mode.REAL : Mode.REPLAY);
    public static final boolean isTuningMode = DriverStation.isTest();

    public static enum Mode {
        /** Running on a real robot. */
        REAL,

        /** Running a physics simulator. */
        SIM,

        /** Replaying from a log file. */
        REPLAY
    }

    public static final double loopPeriodSecs = 0.02; // 50 hz, default loop period
    // public static final Integer AmpConstants = 0;

    public static final String canivore = "CANt_open_file"; //this is what 177 uses, change to your canivores name if you use one



    public static final class FieldConstants {
        //put all season-specific field constants here
        public static final double fieldLength = 0; //meters
        public static final double fieldWidth = 0;
    }



    public static final class CANdleConstants{
        public static final int CANdleID = 0; 
        public static final int LedCount = 8; //    base CANdle without any led strips
    }

    public static final class AimAssistConstants{
        public static final double kP_X = 0.2; //   these will be the same value in most cases
        public static final double kP_Y = 0.2;
        public static final double kP_Theta = 1;
    }

}
