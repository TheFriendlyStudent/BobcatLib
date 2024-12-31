package BobcatLib.Subsystems.Swerve.AdvancedSwerve.Swerve.Interfaces;


import edu.wpi.first.units.Measure;
import edu.wpi.first.units.measure.Voltage;

public interface SysidCompatibleSwerve {
    //TODO: better documentation
    /**
     * set all modules to supplied voltage
     */
    public default void sysidVoltage(Voltage volts){}
    
    /**
     * volts
     */
    public default double getModuleVoltage(int moduleNumber){
        return 0;
    }
    /**
     * meters
     */
    public default double getModuleDistance(int moduleNumber){
        return 0;
    }
    /**
     * meters/sec
     */
    public default double getModuleSpeed(int moduleNumber){
        return 0;
    }
}
