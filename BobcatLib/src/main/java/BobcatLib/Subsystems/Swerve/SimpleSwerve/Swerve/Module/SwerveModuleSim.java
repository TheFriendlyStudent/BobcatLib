package BobcatLib.Subsystems.Swerve.SimpleSwerve.Swerve.Module;

import BobcatLib.Subsystems.Swerve.SimpleSwerve.Swerve.Module.Utility.ModuleConstants;
import BobcatLib.Subsystems.Swerve.SimpleSwerve.Swerve.Module.parser.ModuleJson;
import BobcatLib.Subsystems.Swerve.SimpleSwerve.Swerve.Module.parser.ModuleLimitsJson;
import BobcatLib.Subsystems.Swerve.SimpleSwerve.Utility.math.Conversions;
import BobcatLib.Subsystems.Swerve.Utility.CotsModuleSwerveConstants;
import com.ctre.phoenix6.controls.VoltageOut;
import com.fasterxml.jackson.databind.ObjectMapper;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.StringSubscriber;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import java.io.File;
import java.io.IOException;

public class SwerveModuleSim implements SwerveModuleIO {
  private Rotation2d angleOffset;
  public int moduleNumber;
  private FlywheelSim driveSim;
  private FlywheelSim angleSim;
  private double loopPeriodSecs = 0.2;
  /** NT3 Raw Absolute Angle publisher for the absolute encoder. */
  private final String rawAbsoluteAngleName;
  /** NT3 raw angle motor. */
  private final String rawAngleName;
  /** NT3 Raw drive motor. */
  private final String rawDriveName;
  /* SysId Voltage Control */
  private VoltageOut sysidControl = new VoltageOut(0);

  // Subscribed too the
  public StringSubscriber configSubscriber;
  public String lastConfig = "{}";
  public ModuleJson jsonModule;

  private SwerveModuleState desiredState = new SwerveModuleState();

  public ModuleConstants chosenModule;
  public ModuleLimitsJson swerveLimits;

  public SwerveModuleSim(int moduleNumber, ModuleLimitsJson limits) {
    this.moduleNumber = moduleNumber;
    this.swerveLimits = limits;
    loadConfigurationFromFile();
    configModule();
    // Using flywheels to simulate motors
    driveSim = new FlywheelSim(DCMotor.getFalcon500(1), chosenModule.driveGearRatio, 0.025);
    angleSim = new FlywheelSim(DCMotor.getFalcon500(1), chosenModule.angleGearRatio, 0.004);
    rawAbsoluteAngleName = "Module[" + moduleNumber + "] Raw Absolute Encoder";
    rawAngleName = "Module[" + moduleNumber + "] Raw Angle Encoder";
    rawDriveName = "Module[" + moduleNumber + "] Raw Drive Encoder";
    angleOffset = Rotation2d.fromDegrees(0);
  }

  public ModuleJson loadConfigurationFromFile() {
    String name = "module" + moduleNumber + ".json";
    File deployDirectory = Filesystem.getDeployDirectory();
    assert deployDirectory.exists();
    File directory = new File(deployDirectory, "configs/swerve");
    assert new File(directory, name).exists();
    File moduleFile = new File(directory, name);
    assert moduleFile.exists();
    jsonModule = new ModuleJson();
    try {
      jsonModule = new ObjectMapper().readValue(moduleFile, ModuleJson.class);
      jsonModule.wheelCircumference = Units.inchesToMeters(jsonModule.wheelCircumference);
    } catch (IOException e) {

    }
    return jsonModule;
  }

  public void configModule() {
    CotsModuleSwerveConstants cotsModule;
    switch (jsonModule.type) {
      case "mk4":
        cotsModule =
            CotsModuleSwerveConstants.SDS.MK4.KrakenX60(
                CotsModuleSwerveConstants.SDS.MK4.driveRatios.L2);
        break;
      case "mk4i":
        cotsModule =
            CotsModuleSwerveConstants.SDS.MK4i.KrakenX60(
                CotsModuleSwerveConstants.SDS.MK4i.driveRatios.L2);
        break;
      case "mk4n":
        cotsModule =
            CotsModuleSwerveConstants.SDS.MK4n.KrakenX60(
                CotsModuleSwerveConstants.SDS.MK4n.driveRatios.L2);
        break;
      case "mk4c":
        cotsModule =
            CotsModuleSwerveConstants.SDS.MK4c.KrakenX60(
                CotsModuleSwerveConstants.SDS.MK4c.driveRatios.L2);
        break;
      default:
        cotsModule =
            CotsModuleSwerveConstants.SDS.MK4i.KrakenX60(
                CotsModuleSwerveConstants.SDS.MK4i.driveRatios.L2);
    }
    chosenModule = new ModuleConstants(cotsModule, jsonModule);
  }

  public void updateInputs(SwerveModuleIOInputs inputs) {
    inputs.offset = angleOffset;

    SwerveModuleState state = getState();
    inputs.velocity = state.speedMetersPerSecond;
    inputs.angle = state.angle;
    inputs.absAngle = state.angle;
  }

  /**
   * sets the desired state of the module's angle and drive motors.
   *
   * @param state desired state
   * @param isOpenLoop is open loop
   */
  public void setDesiredState(SwerveModuleState state, boolean isOpenLoop) {
    SwerveModuleState optimizedState = SwerveModuleState.optimize(state, getState().angle);
    setAngle(optimizedState);
    setSpeed(optimizedState, isOpenLoop);
    desiredState = optimizedState;
  }

  /**
   * sets the drive motor's speed given the desired state's velocity in rotations/second
   *
   * @param state desired state
   * @param isOpenLoop is open loop
   */
  private void setSpeed(SwerveModuleState state, boolean isOpenLoop) {
    double percent = state.speedMetersPerSecond;
    double volts = MathUtil.clamp(percent * 12, -12.0, 12.0);
    driveSim.setInputVoltage(volts);
  }

  /**
   * sets the angle motor's position from desired rotations
   *
   * @param desiredState
   */
  private void setAngle(SwerveModuleState state) {
    double percent = state.speedMetersPerSecond;
    double volts = MathUtil.clamp(percent * 12, -12.0, 12.0);
    driveSim.setInputVoltage(volts);
  }

  /**
   * gets the current modules state in velocity and position given drive motor velocity in rotations
   * / sec given angle motor position in rotations
   *
   * @return swerve module state
   */
  public SwerveModuleState getState() {
    double pos = (driveSim.getAngularVelocityRPM() / 60) * loopPeriodSecs;
    return new SwerveModuleState(getVelocityMetersPerSec(), Rotation2d.fromRotations(pos));
  }

  /**
   * Gets the current velocity of the drive motor in meters per second
   *
   * @return velocity, in meter per second
   */
  public double getVelocityMetersPerSec() {
    double vel = driveSim.getAngularVelocityRPM() / 60;
    double wheelCircumference = chosenModule.wheelCircumference;
    return Conversions.RPSToMPS(vel, wheelCircumference);
  }

  /** Stops the motors properly. */
  public void stopMotors() {
    driveSim.setInputVoltage(0);
    angleSim.setInputVoltage(0);
  }

  public int getModuleNumber() {
    return moduleNumber;
  }

  public SwerveModuleState getDesiredState() {
    return desiredState;
  }
}
