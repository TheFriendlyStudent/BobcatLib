package BobcatLib.Subsystems.Swerve.SimpleSwerve.Swerve.Module.parser;

public class ModuleJson {
  public ModuleMotorJson drive;
  public ModuleMotorJson angle;
  public ModuleEncoderJson encoder;
  public String type = "";
  public String level = "";
  public double wheelCircumference = 0.0;
  /** Drive Current Limit */
  public int driveCurrentLimit = 35;
  /** Drive Current Threshold */
  public int driveCurrentThreshold = 60;
  /** Drive Current THreshold Time */
  public double driveCurrentThresholdTime = 0.1;
  /** Drive ENable Current Limit */
  public boolean driveEnableCurrentLimit = true;
  /** Angle Current Limit */
  public int angleCurrentLimit = 25;
  /** Angle Current Threshold */
  public int angleCurrentThreshold = 40;
  /** Angle Current ThresholdTime */
  public double angleCurrentThresholdTime = 0.1;
  /** ANgle Enable Current Limit */
  public boolean angleEnableCurrentLimit = true;
  /** Ioen Loop Ramp */
  public double openLoopRamp = 0.25;
  /** Closed Loop Ramp */
  public double closedLoopRamp = 0.0;
}
