package BobcatLib.Subsystems.Swerve.AdvancedSwerve;

import static edu.wpi.first.units.Units.Volts;

import BobcatLib.Subsystems.Swerve.AdvancedSwerve.Constants.SwerveConstants;
import BobcatLib.Subsystems.Swerve.AdvancedSwerve.Gyro.GyroIO;
import BobcatLib.Subsystems.Swerve.AdvancedSwerve.Gyro.GyroIOPigeon2;
import BobcatLib.Subsystems.Swerve.AdvancedSwerve.Interfaces.AutomatedSwerve;
import BobcatLib.Subsystems.Swerve.AdvancedSwerve.Interfaces.SysidCompatibleSwerve;
import BobcatLib.Subsystems.Swerve.AdvancedSwerve.StandardDeviations.SwerveStdDevs;
import BobcatLib.Subsystems.Swerve.AdvancedSwerve.SwerveModule.SwerveModule;
import BobcatLib.Subsystems.Swerve.AdvancedSwerve.SwerveModule.SwerveModuleIO;
import BobcatLib.Subsystems.Swerve.AdvancedSwerve.SwerveModule.SwerveModuleIOFalcon;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.units.Measure;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.Arrays;
import java.util.List;
import java.util.Optional;
import java.util.concurrent.locks.Lock;
import java.util.concurrent.locks.ReentrantLock;
import org.littletonrobotics.junction.Logger;

public class SwerveBase extends SubsystemBase implements SysidCompatibleSwerve, AutomatedSwerve {

  private final GyroIO gyroIO;
  private final GyroIOInputsAutoLogged gyroInputs = new GyroIOInputsAutoLogged();
  private final SwerveModule[] modules;

  private final double[] swerveModuleStates = new double[8];
  private final double[] desiredSwerveModuleStates = new double[8];

  private Rotation2d ppRotationOverride;

  private final PIDController rotationPID;
  private final PIDController autoAlignPID;
  private double lastMovingYaw = 0.0;
  private boolean rotating = false;

  public static final Lock odometryLock = new ReentrantLock();

  private Rotation2d lastYaw = new Rotation2d();
  private double loopPeriodSecs;
  private int[] filterTags;
  private SwerveConstants constants;

  // aim assist
  private Rotation2d autoAlignAngle = new Rotation2d();
  private Translation2d aimAssistTranslation = new Translation2d();

  private final PathConstraints pathfindingConstraints;

  Matrix<N3, N1> trustautostdDev;
  Matrix<N3, N1> trusttelestdDev;
  Matrix<N3, N1> regautostdDev;
  Matrix<N3, N1> regtelestdDev;

  /**
   * Core constructor for the swerve base, dont use this unless you know what you are doing
   *
   * @param gyroIO the gyro to be used
   * @param flIO the front left swerve module
   * @param frIO the front right swerve module
   * @param blIO the back left swerve module
   * @param brIO the back right swerve module
   * @param loopPeriodSecs processor loop period in seconds (default 0.02)
   * @param filterTags the tags to be ignored by the cameras
   * @param trustautostdDev the trust std devs for auto
   * @param trusttelestdDev the trust std devs for tele
   * @param regautostdDev the regular std devs for auto
   * @param regtelestdDev the regular std devs for tele
   * @param constants the constants for the swerve base
   * @param cameras the cameras to be used for vision
   */
  public SwerveBase(
      GyroIO gyroIO,
      SwerveModuleIO flIO,
      SwerveModuleIO frIO,
      SwerveModuleIO blIO,
      SwerveModuleIO brIO,
      double loopPeriodSecs,
      int[] filterTags,
      Matrix<N3, N1> trustautostdDev,
      Matrix<N3, N1> trusttelestdDev,
      Matrix<N3, N1> regautostdDev,
      Matrix<N3, N1> regtelestdDev,
      SwerveConstants constants,
      PhoenixOdometryThread threadInstance) {

    this.constants = constants;
    this.gyroIO = gyroIO;

    pathfindingConstraints =
        new PathConstraints(
            constants.speedLimits.chassisLimits.maxVelocity,
            constants.speedLimits.chassisLimits.maxAccel,
            constants.speedLimits.chassisLimits.maxAngularVelocity.getRadians(),
            constants.speedLimits.chassisLimits.maxAngularAccel.getRadians());

    modules =
        new SwerveModule[] {
          new SwerveModule(flIO, 0, constants),
          new SwerveModule(frIO, 1, constants),
          new SwerveModule(blIO, 2, constants),
          new SwerveModule(brIO, 3, constants)
        };
    this.loopPeriodSecs = loopPeriodSecs;
    this.filterTags = filterTags;

    threadInstance.start();

    rotationPID =
        new PIDController(
            constants.pidConfigs.teleopConfig.rotKP,
            constants.pidConfigs.teleopConfig.rotKI,
            constants.pidConfigs.teleopConfig.rotKD);
    rotationPID.enableContinuousInput(0, 2 * Math.PI);
    autoAlignPID =
        new PIDController(
            constants.pidConfigs.autoAlignConfig.rotKP,
            constants.pidConfigs.autoAlignConfig.rotKI,
            constants.pidConfigs.autoAlignConfig.rotKD);
    autoAlignPID.enableContinuousInput(0, 2 * Math.PI);

    // std devs will be actually set later, so we dont need to initialize them to actual values here
  }

  // setpointGenerator =
  // SwerveSetpointGenerator.builder()
  // .kinematics(SwerveConstants.swerveKinematics)
  // .moduleLocations(SwerveConstants.moduleTranslations)
  // .build();

  /**
   * @param constants the constants for the swerve base
   * @param filterTags the tags to be ignored by the cameras
   * @param standardDeviations vision measurement std devs
   * @param cameras the cameras to be used for vision
   */
  public SwerveBase(
      SwerveConstants constants,
      int[] filterTags,
      SwerveStdDevs standardDeviations,
      PhoenixOdometryThread threadInstance) {
        new GyroIOPigeon2(0, threadInstance),
        new SwerveModuleIOFalcon( // front left
            constants.moduleConfigs.frontLeft.moduleConstants,
            constants.useFOC,
            constants.pidConfigs.driveMotorConfig,
            constants.pidConfigs.angleMotorConfig,
            cancoderRange,
            cancoderDirection,
            constants.canbus,
            threadInstance),
        new SwerveModuleIOFalcon( // front right
            constants.moduleConfigs.frontRight.moduleConstants,
            constants.useFOC,
            constants.pidConfigs.driveMotorConfig,
            constants.pidConfigs.angleMotorConfig,
            cancoderRange,
            cancoderDirection,
            constants.canbus,
            threadInstance),
        new SwerveModuleIOFalcon( // back left
            constants.moduleConfigs.backLeft.moduleConstants,
            constants.useFOC,
            constants.pidConfigs.driveMotorConfig,
            constants.pidConfigs.angleMotorConfig,
            cancoderRange,
            cancoderDirection,
            constants.canbus,
            threadInstance
  }

  }

  public void setLastMovingYaw(double value) {
    lastMovingYaw = value;
  }

  /** if we are overriding the rotation target, return it, otherwise return an empty optional */
  public Optional<Rotation2d> getRotationTarget() {
    if (getRotationTarget() != null) {
      return Optional.of(getRotationTargetOverride());
    } else {
      return Optional.empty();
    }
  }

  /**
   * the rotation2d this returns will override the one in pathplanner, if null, the default
   * pathplanner rotation will be used
   */
  public Rotation2d getRotationTargetOverride() {
    return ppRotationOverride;
  }

  public void setRotationTarget(Rotation2d target) {
    ppRotationOverride = target;
  }

  @Override
  public void periodic() {
    // Priority IDs should be set in your SEASON SPECIFIC swerve subsystem, NOT in this base
    // subsystem

    odometryLock.lock();
    gyroIO.updateInputs(gyroInputs);
    Logger.processInputs("Swerve/Gyro", gyroInputs);

    for (SwerveModule module : modules) {
      module.periodic();
    }
    odometryLock.unlock();

    Logger.recordOutput("Swerve/YawSetpoint", lastMovingYaw);
    Logger.recordOutput("Swerve/CurrentYaw", getYaw().getRadians());
    Logger.recordOutput("Swerve/Odometry/PureOdom", poseEstimator.getPureOdometry());
    Logger.recordOutput("Swerve/Odometry/State", getOdometryState());

    // Update odometry
    double[] sampleTimestamps = modules[0].getOdometryTimestamps();
    int sampleCount = sampleTimestamps.length;
    for (int i = 0; i < sampleCount; i++) {

      // Read wheel positions from each module
      SwerveModulePosition[] modulePositions = new SwerveModulePosition[4];
      for (int moduleIndex = 0; moduleIndex < 4; moduleIndex++) {
        modulePositions[moduleIndex] = modules[moduleIndex].getOdometryPositions()[i];
      }

      if (gyroInputs.connected) { // Use gyro when connected
        Rotation2d yaw = getYaw();
        lastYaw = yaw;
      } else { // If disconnected or sim, use angular velocity
        Rotation2d yaw =
            lastYaw.plus(
                Rotation2d.fromRadians(getChassisSpeeds().omegaRadiansPerSecond * loopPeriodSecs));
        lastYaw = yaw;
      }

      // determine how much to trust odometry based on acceleration
      Logger.recordOutput("Swerve/OdometryState", getOdometryState());
      switch (getOdometryState()) {
        case THROWOUT:
          break;
        case DISTRUST:
          poseEstimator.updateWithTime(
              sampleTimestamps[i],
              lastYaw,
              modulePositions,
              constants.odometryConstants.distrustStdDevs);
          break;
        case TRUST:
          poseEstimator.updateWithTime(
              sampleTimestamps[i],
              lastYaw,
              modulePositions,
              constants.odometryConstants.trustStdDevs);
          break;
        default:
          poseEstimator.updateWithTime(
              sampleTimestamps[i],
              lastYaw,
              modulePositions,
              constants.odometryConstants.trustStdDevs);
          break;
      }
    }

    // updates desired and current swerve module states
    for (SwerveModule mod : modules) {
      desiredSwerveModuleStates[mod.index * 2 + 1] = mod.getDesiredState().speedMetersPerSecond;
      desiredSwerveModuleStates[mod.index * 2] = mod.getDesiredState().angle.getDegrees();
      swerveModuleStates[mod.index * 2 + 1] = mod.getState().speedMetersPerSecond;
      swerveModuleStates[mod.index * 2] = mod.getState().angle.getDegrees();
    }

    Logger.recordOutput("Swerve/Rotation", getYaw().getDegrees());
    Logger.recordOutput("Swerve/DesiredModuleStates", desiredSwerveModuleStates);
    Logger.recordOutput("Swerve/ModuleStates", swerveModuleStates);
    Logger.recordOutput("Swerve/Pose", getPose());
    Logger.recordOutput(
        "Swerve/ChassisSpeeds",
        new Translation2d(
            ChassisSpeeds.fromRobotRelativeSpeeds(getChassisSpeeds(), getYaw()).vxMetersPerSecond,
            ChassisSpeeds.fromRobotRelativeSpeeds(getChassisSpeeds(), getYaw()).vyMetersPerSecond));

    // stops drivetrain on disable
    if (DriverStation.isDisabled()) {
      for (SwerveModule mod : modules) {
        mod.stop();
      }
    }

    // update pose and configure cameras
    for (Vision camera : cameras) {
      // tells the limelight the orientation of the gyro for calculating pose ambiguity
      camera.SetRobotOrientation(getYaw());

      // updates the pose using megatag 2 algo
      addVisionMG2(camera);

      // tells the cameras which tag to ignore,
      // we do this multiple times because sometimes the code will execute before the LLs are booted
      // up
      if (DriverStation.isDisabled()) {
        camera.setPermittedTags(filterTags);
      }
    }
  }

  @Override
  public void simulationPeriodic() {
    Rotation2d yaw =
        lastYaw.plus(
            Rotation2d.fromRadians(getChassisSpeeds().omegaRadiansPerSecond * loopPeriodSecs));
    lastYaw = yaw;
    switch (getOdometryState()) {
      case THROWOUT:
        break;
      case DISTRUST:
        poseEstimator.update(
            getYaw(), getModulePositions(), constants.odometryConstants.distrustStdDevs);
        break;
      case TRUST:
        poseEstimator.update(
            getYaw(), getModulePositions(), constants.odometryConstants.trustStdDevs);
        break;
      default:
        poseEstimator.update(
            getYaw(), getModulePositions(), constants.odometryConstants.trustStdDevs);
        break;
    }
    poseEstimator.update(getYaw(), getModulePositions());
  }

  /**
   * @return the OdometryState representing how much we should trust the odometry based on
   *     acceleration
   */
  public OdometryState getOdometryState() {
    double avgAccel = 0;
    for (SwerveModule module : modules) {
      if (module.getDriveAcceleration() > 5) {
        return OdometryState.THROWOUT;
      }
      avgAccel += module.getDriveAcceleration() / modules.length;
    }
    Logger.recordOutput("Swerve/Odometry/avgAccel", avgAccel);
    if (avgAccel > 4) {
      return OdometryState.DISTRUST;
    } else {
      return OdometryState.TRUST;
    }
  }

  /**
   * Gets the current yaw of the gyro or the estimated yaw if the gyro is disconnected
   *
   * @return current yaw of the gyro
   */
  public Rotation2d getYaw() {
    if (gyroInputs.connected) { // Use gyro when connected
      return gyroInputs.yawPosition;
    } else { // If disconnected or sim, use angular velocity
      return lastYaw;
    }
  }

  public Rotation2d getWrappedYaw() {
    return BobcatUtil.wrapRot2d(getYaw());
  }

  /**
   * Makes the swerve drive move
   *
   * @param translation desired x and y speeds of the swerve drive in meters per second
   * @param rotation desired rotation speed of the swerve drive in radians per second
   * @param fieldRelative whether the values should be field relative or not
   * @param autoAlign in radians
   */
  public void drive(
      Translation2d translation, double rotation, boolean fieldRelative, boolean autoAlign) {
    boolean rotationOverriden =
        Math.abs(rotation)
            < 0.02; // add a little bit of tolerance for if the stick gets bumped or smth
    autoAlignAngle = BobcatUtil.wrapRot2d(autoAlignAngle());

    ChassisSpeeds desiredSpeeds =
        fieldRelative
            ? ChassisSpeeds.fromFieldRelativeSpeeds(
                translation.getX(), translation.getY(), rotation, getYaw())
            : new ChassisSpeeds(translation.getX(), translation.getY(), rotation);

    if (autoAlign && !rotationOverriden) {
      desiredSpeeds.omegaRadiansPerSecond =
          autoAlignPID.calculate(getWrappedYaw().getRadians(), autoAlignAngle.getRadians());
      lastMovingYaw = getYaw().getRadians();
    } else {
      if (rotation == 0) {
        if (rotating) {
          rotating = false;
          lastMovingYaw = getYaw().getRadians();
        }
        desiredSpeeds.omegaRadiansPerSecond =
            rotationPID.calculate(
                BobcatUtil.get0to2Pi(getYaw().getRadians()), BobcatUtil.get0to2Pi(lastMovingYaw));
      } else {
        rotating = true;
      }
    }

    desiredSpeeds = ChassisSpeeds.discretize(desiredSpeeds, loopPeriodSecs);

    // currentSetpoint =
    // setpointGenerator.generateSetpoint(SwerveConstants.moduleLimits,
    // currentSetpoint, desiredSpeeds, Constants.loopPeriodSecs);

    SwerveModuleState[] swerveModuleStates =
        constants.kinematicsConstants.kinematics.toSwerveModuleStates(desiredSpeeds);
    // SwerveModuleState[] swerveModuleStates = currentSetpoint.moduleStates();
    SwerveDriveKinematics.desaturateWheelSpeeds(
        swerveModuleStates, constants.speedLimits.moduleLimits.maxVelocity);

    for (SwerveModule mod : modules) {
      mod.setDesiredState(swerveModuleStates[mod.index]);
    }
  }

  /**
   * Make the swerve drive move
   *
   * @param targetSpeeds the desired chassis speeds
   */
  public void drive(ChassisSpeeds targetSpeeds) {
    targetSpeeds = ChassisSpeeds.discretize(targetSpeeds, loopPeriodSecs);

    lastMovingYaw = getYaw().getRadians();

    SwerveModuleState[] swerveModuleStates =
        constants.kinematicsConstants.kinematics.toSwerveModuleStates(targetSpeeds);
    SwerveDriveKinematics.desaturateWheelSpeeds(
        swerveModuleStates, constants.speedLimits.moduleLimits.maxVelocity);

    for (SwerveModule mod : modules) {
      mod.setDesiredState(swerveModuleStates[mod.index]);
    }
  }

  /**
   * Sets all of the modules to their desired states
   *
   * @param desiredStates array of states for the modules to be set to
   */
  public void setModuleStates(SwerveModuleState[] desiredStates) {
    SwerveDriveKinematics.desaturateWheelSpeeds(
        desiredStates, constants.speedLimits.moduleLimits.maxVelocity);
    for (SwerveModule mod : modules) {
      mod.setDesiredState(desiredStates[mod.index]);
    }
  }

  /**
   * Gets all of the current module states
   *
   * @return array of the current module states
   */
  public SwerveModuleState[] getModuleStates() {
    SwerveModuleState[] states = new SwerveModuleState[4];
    for (SwerveModule mod : modules) {
      states[mod.index] = mod.getState();
    }
    return states;
  }

  /**
   * Gets all of the current module positions
   *
   * @return array of the current module positions
   */
  public SwerveModulePosition[] getModulePositions() {
    SwerveModulePosition[] positions = new SwerveModulePosition[4];
    for (SwerveModule mod : modules) {
      positions[mod.index] = mod.getPosition();
    }
    return positions;
  }

  /**
   * Gets ths current chassis speeds
   *
   * @return current chassis speeds
   */
  public ChassisSpeeds getChassisSpeeds() {
    return constants.kinematicsConstants.kinematics.toChassisSpeeds(getModuleStates());
  }

  /**
   * Gets the current pose, according to our odometry
   *
   * @return current pose in meters
   */
  public Pose2d getPose() {
    // return odometry.getPoseMeters();
    return poseEstimator.getEstimatedPosition();
  }

  /**
   * Resets our odometry to desired pose
   *
   * @param pose pose to set odometry to
   */
  public void resetPose(Pose2d pose) {
    poseEstimator.resetPosition(getYaw(), getModulePositions(), pose);
  }

  /** Sets the current gyro yaw to 0 degrees */
  public void zeroGyro() {
    setGyro(Rotation2d.fromDegrees(0));
  }

  /**
   * @param angle Gyro Angle
   */
  public void setGyro(Rotation2d angle) {
    gyroIO.setYaw(angle.getDegrees());
    lastMovingYaw = angle.getDegrees();
    lastYaw = Rotation2d.fromDegrees(angle.getDegrees());
  }

  // boolean <- pronnounced 'bolly-un'
  // erm... what the sigma?

  /** Stops the swerve drive */
  public void stop() {
    drive(new ChassisSpeeds());
  }

  public Command driveToPose(Pose2d pose) {
    return AutoBuilder.pathfindToPose(pose, pathfindingConstraints);
  }

  /** does NOT consider alliance color */
  public Translation2d getTranslationToPose(Translation2d pose) {
    return pose.minus(getPose().getTranslation());
  }

  public Translation2d getTranslationToPose(Translation2d bluePose, Translation2d redPose) {
    return BobcatUtil.getAlliance() == Alliance.Blue
        ? bluePose.minus(getPose().getTranslation())
        : redPose.minus(getPose().getTranslation());
  }

  public boolean aligned(AlignmentCheckType checkType) {
    double tolerance = constants.holoAlignTolerance.getRadians();
    switch (checkType) {
      case AUTOALIGN:
        return Math.abs(autoAlignPID.getPositionError()) <= tolerance;
      case BASE_ROTATION:
        return Math.abs(rotationPID.getPositionError()) <= tolerance;
      case PATHPLANNER:
        if (BobcatUtil.getAlliance() == Alliance.Blue) {
          return Math.abs(ppRotationOverride.getRadians() - getYaw().getRadians()) <= tolerance;
        } else {
          return Math.abs(
                  ppRotationOverride.getRadians()
                      - BobcatUtil.get0to2Pi(
                          getYaw().rotateBy(Rotation2d.fromDegrees(180)).getRadians()))
              <= tolerance;
        }
      default:
        return false;
    }
  }

  public boolean aligned(Rotation2d angle) {
    if (BobcatUtil.getAlliance() == Alliance.Blue) {
      return Math.abs(angle.getRadians() - getYaw().getRadians())
          <= constants.holoAlignTolerance.getRadians();
    } else {
      return Math.abs(angle.getRadians() - getYaw().getRadians())
          <= constants.holoAlignTolerance.getRadians();
    }
  }

  public void addVisionMG2(Vision vision) {

    Matrix<N3, N1> stdDev;
    Matrix<N3, N1> truststdDev = DriverStation.isAutonomous() ? trustautostdDev : trusttelestdDev;
    Matrix<N3, N1> regstdDev = DriverStation.isAutonomous() ? regautostdDev : regtelestdDev;
    Logger.recordOutput("Pose/" + vision.getLimelightName(), vision.getBotPoseMG2());

    // stdDev = regstdDev;
    if (vision.tagCount() >= 2) {
      stdDev = truststdDev;
    } else {
      stdDev = regstdDev;
    }

    if (vision.getPoseValidMG2(getYaw())) {
      poseEstimator.addVisionMeasurement(
          vision.getBotPoseMG2(), vision.getPoseTimestampMG2(), stdDev);
      // System.out.println("yes " + vision.getLimelightName() + " " +
      // Timer.getFPGATimestamp());
    }
  }

  public void addVisionNorthStar(VisionObservation visionData) {
    poseEstimator.addVisionMeasurement(
        visionData.getPose(), visionData.getTimestamp(), visionData.getStdDev());
  }

  public enum OdometryState {
    TRUST,
    DISTRUST,
    THROWOUT
  }

  public enum AlignmentCheckType {
    PATHPLANNER,
    AUTOALIGN,
    BASE_ROTATION
  }

  /* sysid stuff */

  /** set all modules to supplied voltage */
  @Override
  public void sysidVoltage(Measure<Voltage> volts) {
    for (SwerveModule mod : modules) {
      mod.runCharachterization(volts.in(Volts));
    }
  }

  /**
   * volts
   *
   * <p>index of module number starts at 0
   */
  @Override
  public double getModuleVoltage(int moduleNumber) {
    return modules[moduleNumber].getVoltage();
  }
  /** meters */
  @Override
  public double getModuleDistance(int moduleNumber) {
    return modules[moduleNumber].getPositionMeters();
  }

  /** meters/sec */
  @Override
  public double getModuleSpeed(int moduleNumber) {
    return modules[moduleNumber].getVelocityMetersPerSec();
  }

  /*end sysid stuff */

  /*aim assist stuff */
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
  /*end aim assist stuff*/
}
