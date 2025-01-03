package BobcatLib.Subsystems.Swerve.AdvancedSwerve.Util;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

public class DSUtil {

    /**
     * 
     * @return alliance, blue if fms is disconnected
     */
    public static Alliance getAlliance(){
        return DriverStation.getAlliance().isPresent() ? DriverStation.getAlliance().get() : Alliance.Blue;
    }

    public static Boolean isBlue(){
        return getAlliance() == Alliance.Blue;
    }
    public static Boolean isRed(){
        return getAlliance() == Alliance.Red;
    }
    public static Boolean isConnected(){
        return DriverStation.getAlliance().isEmpty();
    }
}
