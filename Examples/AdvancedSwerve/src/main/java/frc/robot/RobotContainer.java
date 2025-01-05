// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.io.File;

import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

import com.fasterxml.jackson.databind.ObjectMapper;
import com.pathplanner.lib.config.RobotConfig;

import BobcatLib.Hardware.Controllers.AidenGamepads.EightBitDo;
import BobcatLib.Subsystems.Swerve.AdvancedSwerve.SwerveBase;
import BobcatLib.Subsystems.Swerve.AdvancedSwerve.TeleopSwerve;
import BobcatLib.Subsystems.Swerve.AdvancedSwerve.Assists.AimAssist;
import BobcatLib.Subsystems.Swerve.AdvancedSwerve.Assists.AutoAlign;
import BobcatLib.Subsystems.Swerve.AdvancedSwerve.Constants.SwerveConstantCreator;
import BobcatLib.Subsystems.Swerve.AdvancedSwerve.Constants.SwerveConstants;
import BobcatLib.Subsystems.Swerve.AdvancedSwerve.StandardDeviations.StandardDeviation;
import BobcatLib.Subsystems.Swerve.AdvancedSwerve.StandardDeviations.SwerveStdDevs;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;

public class RobotContainer {

        /* Joysticks + Gamepad */
        private final EightBitDo gp = new EightBitDo(0);

        /* Subsystems */
        public SwerveBase swerve;
        public SwerveConstants swerveConstants;
        public RobotConfig robotConfig;
        public AimAssist aimAssist;
        public AutoAlign autoAlign;

        // public Vision limelight1;
        // public Vision[] cameras;

        /* Commands */

        /* Shuffleboard Inputs */
        private final LoggedDashboardChooser<Command> autoChooser = new LoggedDashboardChooser<>("Auto Choices");

        public RobotContainer() {
                try {
                        // swerveConstants = SwerveConstantCreator.parseConstants(new
                        // File("/src/main/java/frc/robot/Subsystems/Swerve/swerve-config.json"));
                        swerveConstants = SwerveConstantCreator.parseConstants(new File(
                                        Filesystem.getDeployDirectory().toString() + "/swerve-config.json"));
                } catch (Exception e) {
                        e.printStackTrace();
                }
                System.out.println(swerveConstants.pigeonID);

                try {
                        robotConfig = RobotConfig.fromGUISettings();
                } catch (Exception e) {
                        e.printStackTrace();
                }

                switch (Constants.currentMode) {
                        // Real robot, instantiate hardware IO implementations
                        case REAL:
                                swerve = SwerveBase.createSwerve(
                                                swerveConstants,
                                                Constants.filterTags,
                                                Constants.stdDevs,
                                                robotConfig);
                        case SIM:
                                swerve = SwerveBase.createSimSwerve(
                                                swerveConstants,
                                                Constants.filterTags,
                                                Constants.stdDevs,
                                                robotConfig);
                        default:
                                swerve = SwerveBase.createSwerve(
                                                swerveConstants,
                                                new int[] {},
                                                new SwerveStdDevs(
                                                                new StandardDeviation(0, 0, 0, 0),
                                                                new StandardDeviation(0, 0, 0, 0)),
                                                robotConfig);

                }

                configureBindings();
        }

        public boolean autoChooserInitialized() {
                return autoChooser.get() != null;
        }

        /**
         * this should only be called once DS and FMS are attached
         */
        public void configureAutos() {

                /*
                 * Auto Events
                 * 
                 * Names must match what is in PathPlanner
                 * Please give descriptive names
                 */
                // NamedCommands.registerCommand("PathfindingCommand", swerve.driveToPose(new
                // Pose2d()));

                /*
                 * Auto Chooser
                 * 
                 * Names must match what is in PathPlanner
                 * Please give descriptive names
                 */
                autoChooser.addDefaultOption("Do Nothing", Commands.none());
                // autoChooser.addOption("test", new PathPlannerAuto("New Auto"));
        }

        /**
         * IMPORTANT NOTE:
         * When a gamepad value is needed by a command, don't
         * pass the gamepad to the command, instead have the
         * constructor for the command take an argument that
         * is a supplier of the value that is needed. To supply
         * the values, use an anonymous/lambda function like this:
         * 
         * () -> buttonOrAxisValue
         */
        public void configureBindings() {
                swerve.setAimAssistTranslation(new Translation2d(0, 0)); // (x,y) coordinate aim assist will go to
                swerve.setAutoAlignAngle(Rotation2d.fromDegrees(0)); // heading autoalign will face

                aimAssist = new AimAssist(
                                () -> new Translation2d(),
                                gp.b,
                                0,
                                0,
                                0);
                autoAlign = new AutoAlign(
                                () -> new Rotation2d(), // make this the desired position
                                gp.a,
                                0.1,
                                0,
                                0);

                swerve.setDefaultCommand(
                                new TeleopSwerve(swerve,
                                                gp.leftYAxis, // translation (front-back)
                                                gp.leftXAxis, // strafe (left-right)
                                                gp.rightXAxis, // rotation
                                                () -> false, // robot centric
                                                () -> 0.0, // fine strafe
                                                () -> 0.0, // fine translation
                                                aimAssist, // Aim assist
                                                autoAlign, // autoalign
                                                0, // stick deadband [0,1]
                                                swerveConstants));// max angular velocity

                // sysid routines
                gp.start.onTrue(new InstantCommand(() -> swerve.resetPose(new Pose2d())));
                gp.select.onTrue(new InstantCommand(() -> swerve.zeroGyro()));

        }

        public Command getAutonomousCommand() {
                return autoChooser.get();
        }
}