package BobcatLib.Subsystems.Swerve.AdvancedSwerve.Swerve;

import com.ctre.phoenix.sensors.AbsoluteSensorRange;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.pathplanner.lib.config.PIDConstants;

import BobcatLib.Subsystems.Swerve.AdvancedSwerve.Utility.Team254.ModuleConstants;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.math.Vector;

public class SwerveConstants {
    public static final Vector<N3> autostateStdDevs = VecBuilder.fill(0.01, 0.01, Units.degreesToRadians(1));
    public static final double stickDeadband = 0.01;
    public static final Rotation2d holoAlignTolerance = Rotation2d.fromDegrees(0);
    public static final int pigeonID = 0;
    public static final boolean useFOC = true;
    public static final double angleGearRatio = ((150.0 / 7.0) / 1.0);
    public static final double driveGearRatio = (5.36 / 1.0);

    public static class Kinematics {
        public static final double wheelBase = 0.521; // meters
        public static final double trackWidth = 0.521;
        public static final double driveBaseRadius = Math.sqrt(2 * Math.pow(wheelBase / 2, 2));
        public static final double wheelCircumference = Units.inchesToMeters(4) * Math.PI;
        public static final Translation2d[] moduleTranslations = new Translation2d[] {
                new Translation2d(wheelBase / 2.0, trackWidth / 2.0),
                new Translation2d(wheelBase / 2.0, -trackWidth / 2.0),
                new Translation2d(-wheelBase / 2.0, trackWidth / 2.0),
                new Translation2d(-wheelBase / 2.0, -trackWidth / 2.0)
        };
        public static final SwerveDriveKinematics kinematics = new SwerveDriveKinematics(moduleTranslations);
    }

    public static class Limits {
        public static class Chassis {
            public static final double maxSpeed = 4.5; // meters
            public static final double maxAccel = 3;
            public static final Rotation2d maxAngularVelocity = Rotation2d.fromDegrees(360 * 2.5);
            public static final Rotation2d maxAngularAccel = Rotation2d.fromDegrees(360 / 2);
        }

        public static class Module {
            public static final double maxSpeed = 5.5;
            public static final double maxAccel = 3;
            public static final double maxAngularVelocity = 5;
            public static final double maxAngularAccel = 5;
        }
    }

    /**
     * also contains gains
     */
    public static class Configs {
        public static class Auto {
            public static final double rotKP = 4;
            public static final double rotKI = 0;
            public static final double rotKD = 0;
            public static final double transKP = 6;
            public static final double transKI = 0;
            public static final double transKD = 1.4;
            public static final PIDConstants transPidConstants = new PIDConstants(transKP, transKI, transKD);
            public static final PIDConstants rotPidConstants = new PIDConstants(rotKP, rotKI, rotKD);
        }

        public static class Teleop {
            public static final double rotKP = 2;
            public static final double rotKI = 0;
            public static final double rotKD = 0;
            public static final double transKP = 10;
            public static final double transKI = 0;
            public static final double transKD = 0;
            public static final PIDConstants transPidConstants = new PIDConstants(transKP, transKI, transKD);
            public static final PIDConstants rotPidConstants = new PIDConstants(rotKP, rotKI, rotKD);
        }

        public static class AutoAlign {
            public static final Rotation2d tolerance = Rotation2d.fromDegrees(2);
            public static final double rotationKP = 3.5;
            public static final double rotationKI = 0;
            public static final double rotationKD = 0;
            public static final PIDConstants rotPidConstants = new PIDConstants(rotationKP, rotationKI, rotationKD);
        }

        public static class Module {
            public static class Angle {
                public static final double kP = 0.3;
                public static final double kI = 0;
                public static final double kD = 0;
                public static final double kS = 0;
                public static final double kV = 0;
                public static final double kA = 0;
                public static final InvertedValue angleMotorInvert = InvertedValue.Clockwise_Positive;
                public static final NeutralModeValue angleNeutralMode = NeutralModeValue.Coast;
                public static final boolean angleSupplyCurrentLimitEnable = true;
                public static final double angleSupplyCurrentLimit = 25.0;
                public static final double angleSupplyCurrentThreshold = 40.0;
                public static final double angleSupplyTimeThreshold = 0.1;
                public static final boolean angleStatorCurrentLimitEnable = true;
                public static final double angleStatorCurrentLimit = 20;
            }

            public static class Drive {
                public static final double kS = 0.14267 / 12;
                public static final double kA = 2.0718 / 12;
                public static final double kV = 0.42622 / 12;
                public static final double kP = 0.1;
                public static final double kI = 0;
                public static final double kD = 0;
                public static final InvertedValue driveMotorInvert = InvertedValue.CounterClockwise_Positive;
                public static final NeutralModeValue driveNeutralMode = NeutralModeValue.Brake;
                public static final boolean driveSupplyCurrentLimitEnable = true;
                public static final double driveSupplyCurrentLimit = 35.0;
                public static final double driveSupplyCurrentThreshold = 60.0;
                public static final double driveSupplyTimeThreshold = 0.1;
                public static final boolean driveStatorCurrentLimitEnable = true;
                public static final double driveStatorCurrentLimit = 60;
                public static final double openLoopRamp = 0.25;
                public static final double closedLoopRamp = 0.0;
            }

            public static class CANCoder {
                public static final AbsoluteSensorRange sensorRange = AbsoluteSensorRange.Unsigned_0_to_360;
                public static final SensorDirectionValue sensorDirection = SensorDirectionValue.CounterClockwise_Positive;
            }
        }
    }

    public static class Odometry {
        // odometry stuff only, vision stuff is in LimelightConstants

        public static final Matrix<N3, N1> trustStdDevs = VecBuilder.fill(0.01, 0.01, Units.degreesToRadians(1));
        public static final Matrix<N3, N1> distrustStdDevs = VecBuilder.fill(0.15, 0.15, Units.degreesToRadians(1));
        public static final double odometryThrowoutAccel = 5.5; // m/s^2
        public static final double odometryDistrustAccel = 4; // m/s^2
    }

    public static class Module {
        /*
         * Offsets must be done with bevels facing towards spivit motor
         */
        /* FRONT LEFT */
        public static final class Module0Constants {
            public static final int cancoderID = 1;
            public static final int angleMotorID = 2;
            public static final int driveMotorID = 1;

            public static final Rotation2d angleOffset = Rotation2d.fromDegrees(285.46875);

            public static final ModuleConstants constants = new ModuleConstants(driveMotorID, angleMotorID, cancoderID,
                    angleOffset);
        }

        /* FRONT RIGHT */
        public static final class Module1Constants {
            public static final int cancoderID = 2;
            public static final int angleMotorID = 4;
            public static final int driveMotorID = 3;

            public static final Rotation2d angleOffset = Rotation2d.fromDegrees(27.861328125); 
            public static final ModuleConstants constants = new ModuleConstants(driveMotorID, angleMotorID, cancoderID,
                    angleOffset);
        }

        /* BACK LEFT */
        public static final class Module2Constants {
            public static final int cancoderID = 3;
            public static final int angleMotorID = 6;
            public static final int driveMotorID = 5;
            public static final Rotation2d angleOffset = Rotation2d.fromDegrees(176.30859375); 

            public static final ModuleConstants constants = new ModuleConstants(driveMotorID, angleMotorID, cancoderID,
                    angleOffset);
        }

        /* BACK RIGHT */
        public static final class Module3Constants {
            public static final int cancoderID = 4;
            public static final int angleMotorID = 8;
            public static final int driveMotorID = 7;
            public static final Rotation2d angleOffset = Rotation2d.fromDegrees(321.153);

            public static final ModuleConstants constants = new ModuleConstants(driveMotorID, angleMotorID, cancoderID,
                    angleOffset);
        }
    }

}
