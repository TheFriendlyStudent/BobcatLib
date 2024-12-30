package BobcatLib.Subsystems.Swerve.SimpleSwerve;

import BobcatLib.Hardware.Gyros.BaseGyro;
import BobcatLib.Hardware.Gyros.GyroSim;
import BobcatLib.Hardware.Gyros.Pigeon2Gyro;
import BobcatLib.Subsystems.Swerve.SimpleSwerve.Swerve.Module.SwerveModule;
import BobcatLib.Subsystems.Swerve.SimpleSwerve.Swerve.Module.SwerveModuleReal;
import BobcatLib.Subsystems.Swerve.SimpleSwerve.Swerve.Module.Utility.PIDConstants;
import BobcatLib.Subsystems.Swerve.SimpleSwerve.Swerve.Module.Utility.Pose.PoseLib;
import BobcatLib.Subsystems.Swerve.SimpleSwerve.Swerve.Module.Utility.Pose.WpiPoseEstimator;
import BobcatLib.Subsystems.Swerve.SimpleSwerve.Swerve.Module.parser.ModuleLimitsJson;
import BobcatLib.Subsystems.Swerve.SimpleSwerve.Swerve.Parser.BaseJson;
import BobcatLib.Subsystems.Swerve.SimpleSwerve.Swerve.Parser.SwerveIndicatorJson;
import BobcatLib.Subsystems.Swerve.SimpleSwerve.Swerve.Parser.SwerveJson;
import BobcatLib.Subsystems.Swerve.SimpleSwerve.Swerve.Parser.chassisLimitsJson;
import BobcatLib.Subsystems.Swerve.SimpleSwerve.Swerve.Parser.driveCharacteristicsJson;
import BobcatLib.Subsystems.Swerve.SimpleSwerve.Swerve.Parser.drivePIDJson;
import BobcatLib.Subsystems.Swerve.SimpleSwerve.Swerve.interfaces.AutomatedSwerve;
import BobcatLib.Subsystems.Swerve.SimpleSwerve.Swerve.interfaces.SysidCompatibleSwerve;
import BobcatLib.Subsystems.Swerve.SimpleSwerve.Utility.Alliance;
import BobcatLib.Subsystems.Swerve.SimpleSwerve.Utility.math.GeometryUtils;
import com.fasterxml.jackson.databind.ObjectMapper;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.VoltageUnit;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.io.File;
import java.io.IOException;
import java.util.concurrent.locks.Lock;
import java.util.concurrent.locks.ReentrantLock;

public class SwerveDrive extends SubsystemBase implements SysidCompatibleSwerve, AutomatedSwerve {
  public SwerveModule[] mSwerveMods;

  private final BaseGyro gyro;
  // Automated Interface Properties
  private Rotation2d autoAlignAngle = new Rotation2d();
  private Translation2d aimAssistTranslation = new Translation2d();
  public boolean isSim = false;
  private final PathConstraints pathfindingConstraints;
  /* Drivetrain Constants */
  public double trackWidth;
  public double wheelBase;
  public SwerveJson jsonSwerve = new SwerveJson();

  public PIDConstants pidTranslation;
  public PIDConstants pidRotation;
  static final Lock odometryLock = new ReentrantLock();
  public PoseLib swerveDrivePoseEstimator;
  private Alliance team;
  Matrix<N3, N1> visionStdDevs;
  Matrix<N3, N1> stateStdDevs;

  /*
   * Swerve Kinematics
   * No need to ever change this unless you are not doing a traditional
   * rectangular/square 4 module swerve
   */
  public static SwerveDriveKinematics swerveKinematics;

  public SwerveDrive(
      boolean isSim, Alliance team, Matrix<N3, N1> visionStdDevs, Matrix<N3, N1> stateStdDevs) {
    this.team = team;
    this.isSim = isSim;
    this.visionStdDevs = visionStdDevs;
    this.stateStdDevs = stateStdDevs;

    /* Drivetrain Constants */
    loadConfigurationFromFile();
    trackWidth = Units.inchesToMeters(jsonSwerve.base.trackWidth);
    wheelBase = Units.inchesToMeters(jsonSwerve.base.wheelBase);
    double maxAngularAccel = jsonSwerve.moduleSpeedLimits.maxAngularAcceleration;
    double maxAngularVelocity = jsonSwerve.moduleSpeedLimits.maxAngularVelocity;
    pathfindingConstraints =
        new PathConstraints(
            jsonSwerve.moduleSpeedLimits.maxSpeed,
            jsonSwerve.moduleSpeedLimits.maxAccel,
            maxAngularVelocity,
            maxAngularAccel);
    swerveKinematics =
        new SwerveDriveKinematics(
            new Translation2d(wheelBase / 2.0, trackWidth / 2.0),
            new Translation2d(wheelBase / 2.0, -trackWidth / 2.0),
            new Translation2d(-wheelBase / 2.0, trackWidth / 2.0),
            new Translation2d(-wheelBase / 2.0, -trackWidth / 2.0));
    /* Setup Modules */
    if (isSim) {
      gyro = new BaseGyro("Swerve-Gyro", new GyroSim());

      mSwerveMods = new SwerveModule[] {};
    } else {
      gyro = new BaseGyro("Swerve-Gyro", new Pigeon2Gyro());
      mSwerveMods =
          new SwerveModule[] {
            new SwerveModule(new SwerveModuleReal(0, jsonSwerve.moduleSpeedLimits), 0),
            new SwerveModule(new SwerveModuleReal(1, jsonSwerve.moduleSpeedLimits), 1),
            new SwerveModule(new SwerveModuleReal(2, jsonSwerve.moduleSpeedLimits), 2),
            new SwerveModule(new SwerveModuleReal(3, jsonSwerve.moduleSpeedLimits), 3)
          };
    }
    Timer.delay(1);
    resetModulesToAbsolute();

    swerveDrivePoseEstimator =
        new WpiPoseEstimator(
            swerveKinematics,
            gyro.getYaw(),
            getModulePositions(),
            new Pose2d()); // x,y,heading in radians; Vision measurement std dev, higher=less
    // weight

    pidTranslation =
        new PIDConstants(
            jsonSwerve.translationPID.driveKP,
            jsonSwerve.translationPID.driveKI,
            jsonSwerve.translationPID.driveKD); // Translation
    pidRotation =
        new PIDConstants(
            jsonSwerve.rotationPID.driveKP,
            jsonSwerve.rotationPID.driveKI,
            jsonSwerve.rotationPID.driveKD); // Rotation
  }

  public SwerveDrive(boolean isSim, Alliance team) {
    this.team = team;
    this.isSim = isSim;
    this.visionStdDevs = null;
    this.stateStdDevs = null;

    /* Drivetrain Constants */
    loadConfigurationFromFile();
    trackWidth = Units.inchesToMeters(jsonSwerve.base.trackWidth);
    wheelBase = Units.inchesToMeters(jsonSwerve.base.wheelBase);
    double maxAngularAccel = jsonSwerve.moduleSpeedLimits.maxAngularAcceleration;
    double maxAngularVelocity = jsonSwerve.moduleSpeedLimits.maxAngularVelocity;
    pathfindingConstraints =
        new PathConstraints(
            jsonSwerve.moduleSpeedLimits.maxSpeed,
            jsonSwerve.moduleSpeedLimits.maxAccel,
            maxAngularVelocity,
            maxAngularAccel);
    swerveKinematics =
        new SwerveDriveKinematics(
            new Translation2d(wheelBase / 2.0, trackWidth / 2.0),
            new Translation2d(wheelBase / 2.0, -trackWidth / 2.0),
            new Translation2d(-wheelBase / 2.0, trackWidth / 2.0),
            new Translation2d(-wheelBase / 2.0, -trackWidth / 2.0));
    /* Setup Modules */
    if (isSim) {
      gyro = new BaseGyro("Swerve-Gyro", new GyroSim());

      mSwerveMods = new SwerveModule[] {};
    } else {
      gyro = new BaseGyro("Swerve-Gyro", new Pigeon2Gyro());
      mSwerveMods =
          new SwerveModule[] {
            new SwerveModule(new SwerveModuleReal(0, jsonSwerve.moduleSpeedLimits), 0),
            new SwerveModule(new SwerveModuleReal(1, jsonSwerve.moduleSpeedLimits), 1),
            new SwerveModule(new SwerveModuleReal(2, jsonSwerve.moduleSpeedLimits), 2),
            new SwerveModule(new SwerveModuleReal(3, jsonSwerve.moduleSpeedLimits), 3)
          };
    }
    Timer.delay(1);
    resetModulesToAbsolute();

    
    swerveDrivePoseEstimator =
        new WpiPoseEstimator(
            swerveKinematics,
            getGyroYaw(),
            getModulePositions(),
            new Pose2d(
                new Translation2d(0, 0),
                Rotation2d.fromDegrees(
                    0))); // x,y,heading in radians; Vision measurement std dev, higher=less weight

    pidTranslation =
        new PIDConstants(
            jsonSwerve.translationPID.driveKP,
            jsonSwerve.translationPID.driveKI,
            jsonSwerve.translationPID.driveKD); // Translation
    pidRotation =
        new PIDConstants(
            jsonSwerve.rotationPID.driveKP,
            jsonSwerve.rotationPID.driveKI,
            jsonSwerve.rotationPID.driveKD); // Rotation
  }

  public SwerveJson loadConfigurationFromFile() {
    File deployDirectory = Filesystem.getDeployDirectory();
    assert deployDirectory.exists();
    File directory = new File(deployDirectory, "configs/swerve");
    assert new File(directory, "swerve.json").exists();
    File swerveFile = new File(directory, "swerve.json");
    assert swerveFile.exists();
    jsonSwerve = new SwerveJson();
    try {
      jsonSwerve = new ObjectMapper().readValue(swerveFile, SwerveJson.class);
      SmartDashboard.putNumber(
          "Swerve ChassisLimits maxSpeed", jsonSwerve.chassisSpeedLimits.maxSpeed);
      SmartDashboard.putNumber(
          "Swerve ModuleLimits maxSpeed", jsonSwerve.moduleSpeedLimits.maxSpeed);
      SmartDashboard.putString(
          "Swerve Base", jsonSwerve.base.trackWidth + " x " + jsonSwerve.base.wheelBase);
      SmartDashboard.putNumber("Swerve Indicator Id", jsonSwerve.indicator.id);
      SmartDashboard.putNumber("Swerve Rotation PID", jsonSwerve.rotationPID.driveKP);
      SmartDashboard.putNumber("Swerve Transalation PID", jsonSwerve.translationPID.driveKP);

    } catch (IOException e) {
      jsonSwerve.base = new BaseJson();
      jsonSwerve.chassisSpeedLimits = new chassisLimitsJson();
      jsonSwerve.driveCharacteristics = new driveCharacteristicsJson();
      jsonSwerve.moduleSpeedLimits = new ModuleLimitsJson();
      jsonSwerve.rotationPID = new drivePIDJson();
      jsonSwerve.translationPID = new drivePIDJson();
      jsonSwerve.indicator = new SwerveIndicatorJson();
    }
    return jsonSwerve;
  }

  public Command driveAsCommand(Translation2d translation) {
    Rotation2d currentHeading = new Rotation2d();
    Pose2d currentPose = new Pose2d();
    Command driveCmd =
        new InstantCommand(() -> drive(translation, 0, true, false, currentHeading, currentPose));
    return driveCmd;
  }

  /**
   * Make the swerve drive move
   *
   * @param targetSpeeds the desired chassis speeds
   */
  public void drive(ChassisSpeeds targetSpeeds) {
    SwerveModuleState[] swerveModuleStates = swerveKinematics.toSwerveModuleStates(targetSpeeds);
    double maxSpeed = jsonSwerve.moduleSpeedLimits.maxSpeed;
    SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, maxSpeed);
    for (SwerveModule mod : mSwerveMods) {
      mod.setDesiredState(swerveModuleStates[mod.getModuleNumber()], true);
    }
  }

  /**
   * drive the swerve with 1st or 2nd order.
   *
   * @param translation
   * @param rotation
   * @param fieldRelative
   * @param isOpenLoop
   */
  public void drive(
      Translation2d translation,
      double rotation,
      boolean fieldRelative,
      boolean isOpenLoop,
      Rotation2d currentHeading,
      Pose2d currentPose) {
    currentHeading = getHeading();
    currentPose = getPose();
    if (Constants.SwerveConstants.firstOrderDriving) {
      drive1stOrder(translation, rotation, fieldRelative, isOpenLoop, currentHeading, currentPose);
    } else {
      drive2ndOrder(translation, rotation, fieldRelative, isOpenLoop, currentHeading, currentPose);
    }
  }

  /**
   * Drive using second order
   *
   * @param translation
   * @param rotation
   * @param fieldRelative
   * @param isOpenLoop
   */
  public void drive1stOrder(
      Translation2d translation,
      double rotation,
      boolean fieldRelative,
      boolean isOpenLoop,
      Rotation2d currentHeading,
      Pose2d currentPose) {
    SwerveModuleState[] swerveModuleStates =
        swerveKinematics.toSwerveModuleStates(
            fieldRelative
                ? ChassisSpeeds.fromFieldRelativeSpeeds(
                    translation.getX(), translation.getY(), rotation, currentHeading)
                : new ChassisSpeeds(translation.getX(), translation.getY(), rotation));
    double maxSpeed = jsonSwerve.moduleSpeedLimits.maxSpeed;
    SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, maxSpeed);
    applyModuleStates(swerveModuleStates, isOpenLoop);
  }

  /**
   * Drive using second order
   *
   * @param translation
   * @param rotation
   * @param fieldRelative
   * @param isOpenLoop
   */
  public void drive2ndOrder(
      Translation2d translation,
      double rotation,
      boolean fieldRelative,
      boolean isOpenLoop,
      Rotation2d currentHeading,
      Pose2d currentPose) {
    ChassisSpeeds desiredChassisSpeeds =
        fieldRelative
            ? ChassisSpeeds.fromFieldRelativeSpeeds(
                translation.getX(),
                translation.getY(),
                rotation,
                currentPose
                    .getRotation()
                    .plus(
                        Rotation2d.fromDegrees(team.get() == DriverStation.Alliance.Red ? 180 : 0)))
            : new ChassisSpeeds(translation.getX(), translation.getY(), rotation);
    desiredChassisSpeeds = correctForDynamics(desiredChassisSpeeds);
    SwerveModuleState[] swerveModuleStates =
        swerveKinematics.toSwerveModuleStates(desiredChassisSpeeds);
    double maxSpeed = jsonSwerve.moduleSpeedLimits.maxSpeed;
    SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, maxSpeed);
    applyModuleStates(swerveModuleStates, isOpenLoop);
  }

  /**
   * Applies the current desired states to the modules.
   *
   * @param swerveModuleStates
   * @param isOpenLoop
   */
  private void applyModuleStates(SwerveModuleState[] swerveModuleStates, boolean isOpenLoop) {
    for (SwerveModule mod : mSwerveMods) {
      mod.setDesiredState(swerveModuleStates[mod.getModuleNumber()], isOpenLoop);
    }
  }

  /**
   * sets the swerve's desired states
   *
   * @param desiredStates
   */
  public void setModuleStates(SwerveModuleState[] desiredStates) {
    double maxSpeed = jsonSwerve.moduleSpeedLimits.maxSpeed;
    SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, maxSpeed);
    applyModuleStates(desiredStates, false);
  }

  /**
   * Gets the module states
   *
   * @return
   */
  public SwerveModuleState[] getModuleStates() {
    SwerveModuleState[] states = new SwerveModuleState[4];
    for (SwerveModule mod : mSwerveMods) {
      states[mod.getModuleNumber()] = mod.getState();
    }
    return states;
  }

  /**
   * Gets the desired module states
   *
   * @return
   */
  public SwerveModuleState[] getDesiredModuleStates() {
    SwerveModuleState[] states = new SwerveModuleState[4];
    for (SwerveModule mod : mSwerveMods) {
      states[mod.getModuleNumber()] = mod.getDesiredState();
    }
    return states;
  }

  /**
   * Gets the module position
   *
   * @return
   */
  public SwerveModulePosition[] getModulePositions() {
    SwerveModulePosition[] positions = new SwerveModulePosition[4];
    for (SwerveModule mod : mSwerveMods) {
      positions[mod.getModuleNumber()] = mod.getPosition();
    }
    return positions;
  }

  /**
   * @return
   */
  public Rotation2d getGyroYaw() {
    return gyro.getYaw();
  }

  /**
   * Read the modules absolute encoder and set the internal steer motors too the absoluate. angle
   * set in units of rotations.
   */
  public void resetModulesToAbsolute() {
    for (SwerveModule mod : mSwerveMods) {
      mod.resetToAbsolute();
    }
  }

  /**
   * Correct drift dynamically
   *
   * @param originalSpeeds
   * @return
   */
  private static ChassisSpeeds correctForDynamics(ChassisSpeeds originalSpeeds) {
    final double LOOP_TIME_S = 0.02;
    Pose2d futureRobotPose =
        new Pose2d(
            originalSpeeds.vxMetersPerSecond * LOOP_TIME_S,
            originalSpeeds.vyMetersPerSecond * LOOP_TIME_S,
            Rotation2d.fromRadians(originalSpeeds.omegaRadiansPerSecond * LOOP_TIME_S));
    Twist2d twistForPose = GeometryUtils.log(futureRobotPose);
    ChassisSpeeds updatedSpeeds =
        new ChassisSpeeds(
            twistForPose.dx / LOOP_TIME_S,
            twistForPose.dy / LOOP_TIME_S,
            twistForPose.dtheta / LOOP_TIME_S);
    return updatedSpeeds;
  }

  /**
   * Gets the speeds from the modules as defined as chassis.
   *
   * @return ChassisSpeeds
   */
  public ChassisSpeeds getChassisSpeeds() {
    // Simple Second Order Kinematics
    ChassisSpeeds internalChassisSpeeds = swerveKinematics.toChassisSpeeds(getModuleStates());
    return correctForDynamics(internalChassisSpeeds);
  }

  @Override
  public void periodic() {
    updateGyroInSim();
    gyro.periodic();
    for (SwerveModule mod : mSwerveMods) {
      mod.periodic();
    }
    // swerveDrivePoseEstimator.setStateStdDevs(new state std devs here);
    swerveDrivePoseEstimator.updateWithTime(
        Timer.getFPGATimestamp(), getGyroYaw(), getModulePositions());
    // swerveDrivePoseEstimator.addVisionMeasurement();
  }

  /** Checks if in sim. */
  private void updateGyroInSim() {
    if (!isSim) {
      return;
    }

    double angle = getChassisSpeeds().omegaRadiansPerSecond * gyro.getTimeDiff();
    gyro.setYaw(angle);
  }

  /** Stops the motors properly. */
  public void stopMotors() {
    for (SwerveModule mod : mSwerveMods) {
      mod.stopMotors();
    }
  }

  /**
   * Add vision measurement to swerve drivetrain "this should be moved into the vision specific
   * stuff." - AO
   *
   * @param pose
   */
  public Command driveToPose(Pose2d pose) {
    return AutoBuilder.pathfindToPose(pose, pathfindingConstraints);
  }

  /* Automated Swerve Classes */
  @Override
  public Rotation2d autoAlignAngle() {
    return autoAlignAngle;
  }

  @Override
  public void setAutoAlignAngle(Rotation2d angle) {
    autoAlignAngle = angle;
  }

  @Override
  public Translation2d aimAssistTranslation() {
    return aimAssistTranslation;
  }

  @Override
  public void setAimAssistTranslation(Translation2d translation) {
    aimAssistTranslation = translation;
  }
  /* end aim assist stuff */

  /* sysid stuff */

  /** set all modules to supplied voltage */
  @Override
  public void sysidVoltage(Measure<VoltageUnit> volts) {
    for (SwerveModule mod : mSwerveMods) {
      mod.runCharachterization(volts.magnitude());
    }
  }

  /**
   * volts
   *
   * <p>index of module number starts at 0
   */
  @Override
  public double getModuleVoltage(int moduleNumber) {
    return mSwerveMods[moduleNumber].getVoltage();
  }

  /** meters */
  @Override
  public double getModuleDistance(int moduleNumber) {
    return mSwerveMods[moduleNumber].getPositionMeters();
  }

  /** meters/sec */
  @Override
  public double getModuleSpeed(int moduleNumber) {
    return mSwerveMods[moduleNumber].getVelocityMetersPerSec();
  }

  /* end sysid stuff */

  /* Odometry / Pose items */

  /**
   * Fetch the latest odometry heading, should be trusted over {@link SwerveDrive#getGyroYaw()}.
   *
   * @return {@link Rotation2d} of the robot heading.
   */
  public Rotation2d getHeading() {
    return swerveDrivePoseEstimator.getEstimatedPosition().getRotation();
  }

  /**
   * Gets the pose from the swerve pose.
   *
   * @return
   */
  public Pose2d getPose() {
    odometryLock.lock();
    Pose2d poseEstimation = swerveDrivePoseEstimator.getEstimatedPosition();
    odometryLock.unlock();
    return poseEstimation;
  }

  /**
   * Sets the pose from the swerve system using gyro.
   *
   * @param pose
   */
  public void setPose(Pose2d pose) {
    odometryLock.lock();
    swerveDrivePoseEstimator.resetPosition(getGyroYaw(), getModulePositions(), pose);
    odometryLock.unlock();
    swerveKinematics.toSwerveModuleStates(
        ChassisSpeeds.fromFieldRelativeSpeeds(0, 0, 0, getGyroYaw()));
  }

  /** Zeros the heading of the swerve based on the gyro */
  public void zeroHeading() {
    odometryLock.lock();
    swerveDrivePoseEstimator.resetPosition(getGyroYaw(), getModulePositions(), getPose());
    odometryLock.unlock();
    swerveKinematics.toSwerveModuleStates(
        ChassisSpeeds.fromFieldRelativeSpeeds(0, 0, 0, getGyroYaw()));
  }
}
