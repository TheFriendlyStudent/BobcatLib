package BobcatLib.Subsystems.Swerve.AdvancedSwerve.Sysid;

import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.MutDistance;
import edu.wpi.first.units.measure.MutLinearVelocity;
import edu.wpi.first.units.measure.MutVoltage;
import edu.wpi.first.units.MutableMeasure;
import edu.wpi.first.units.measure.Velocity;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.sysid.SysIdRoutineLog;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;

import static edu.wpi.first.units.Units.Meter;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Volts;

import BobcatLib.Subsystems.Swerve.AdvancedSwerve.Swerve.Interfaces.SysidCompatibleSwerve;


public class Sysid extends SubsystemBase {
        public enum SysidTest {
            QUASISTATIC_FORWARD,
            QUASISTATIC_BACKWARD,
            DYNAMIC_FORWARD,
            DYNAMIC_BACKWARD
        }
        private SysidCompatibleSwerve swerve;
        private SysIdRoutine routine;
        // Mutable holder for unit-safe voltage values, persisted to avoid reallocation.
        private final MutVoltage m_appliedVoltage = new MutVoltage(0, 0, Volts);
        // Mutable holder for unit-safe linear distance values, persisted to avoid
        // reallocation.
        private final MutDistance m_distance = new MutDistance(0, 0, Meters);
        // Mutable holder for unit-safe linear velocity values, persisted to avoid
        // reallocation.
        private final MutLinearVelocity m_velocity = new MutLinearVelocity(0, 0, MetersPerSecond);

        public Sysid(SysidCompatibleSwerve swerve) {
                this.swerve = swerve;
                routine = new SysIdRoutine(new SysIdRoutine.Config(),
                                new SysIdRoutine.Mechanism(
                                                (swerve::sysidVoltage),
                                                this::logMotors,
                                                this));
        }

    private void logMotors(SysIdRoutineLog log){
        log.motor("front-left")
                                .voltage(m_appliedVoltage.mut_replace(swerve.getModuleVoltage(0) * RobotController.getBatteryVoltage(), Volts))
                                .linearPosition(m_distance.mut_replace(swerve.getModuleDistance(0), Meters))
                                .linearVelocity(m_velocity.mut_replace(swerve.getModuleSpeed(0), MetersPerSecond));
        log.motor("front-right")
                                .voltage(m_appliedVoltage.mut_replace(swerve.getModuleVoltage(1) * RobotController.getBatteryVoltage(),Volts))
                                .linearPosition(m_distance.mut_replace(swerve.getModuleDistance(1), Meters))
                                .linearVelocity(m_velocity.mut_replace(swerve.getModuleSpeed(1),MetersPerSecond));
        log.motor("back-left")
                                .voltage(m_appliedVoltage.mut_replace(swerve.getModuleVoltage(2) * RobotController.getBatteryVoltage(),Volts))
                                .linearPosition(m_distance.mut_replace(swerve.getModuleDistance(2), Meters))
                                .linearVelocity(m_velocity.mut_replace(swerve.getModuleSpeed(2),MetersPerSecond));
        log.motor("back-right")
                                .voltage(m_appliedVoltage.mut_replace(swerve.getModuleVoltage(3) * RobotController.getBatteryVoltage(),Volts))
                                .linearPosition(m_distance.mut_replace(swerve.getModuleDistance(3), Meters))
                                .linearVelocity(m_velocity.mut_replace(swerve.getModuleSpeed(3),MetersPerSecond));

}

    /**
     * REMEMBER TO ENSURE ALL MODULES ARE POINTING FORWARD
     */
    public Command getSysidTest(SysidTest test){
        switch (test) {
            case QUASISTATIC_FORWARD:
                return routine.quasistatic(Direction.kForward);
            case QUASISTATIC_BACKWARD:
                return routine.quasistatic(Direction.kReverse);
            case DYNAMIC_FORWARD:
                return routine.dynamic(Direction.kForward);
            case DYNAMIC_BACKWARD:
                return routine.dynamic(Direction.kReverse);
            default:
                return routine.quasistatic(Direction.kForward);
        }
    }

}
