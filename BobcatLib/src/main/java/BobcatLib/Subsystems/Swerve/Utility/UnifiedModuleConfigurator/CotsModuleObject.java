package BobcatLib.Subsystems.Swerve.Utility.UnifiedModuleConfigurator;

import BobcatLib.Hardware.Motors.SensorHelpers.InvertedWrapper;
import BobcatLib.Hardware.Motors.SensorHelpers.SensorDirectionWrapper;
import BobcatLib.Subsystems.Swerve.Utility.CotsModuleSwerveConstants;
import com.fasterxml.jackson.databind.ObjectMapper;
import edu.wpi.first.wpilibj.Filesystem;
import java.io.File;
import java.io.IOException;

public class CotsModuleObject {
  /** wheelDiameter */
  public double wheelDiameter;
  /** wheelCircumference */
  public double wheelCircumference;
  /** driveGearRatio */
  public double angleGearRatio;
  /** wheelDiameter */
  public double driveGearRatio;
  /** angleKP */
  public double angleKP;
  /** angleKI */
  public double angleKI;
  /** angleKD */
  public double angleKD;
  /** driveMotorInvert */
  public InvertedWrapper driveMotorInvert;
  /** angleMotorInvert */
  public InvertedWrapper angleMotorInvert;
  /** absoluteEncoderInvert */
  public SensorDirectionWrapper absoluteEncoderInvert;

  public CotsModuleObject() {}

  public CotsModuleObject(
      double wheelDiameter,
      double angleGearRatio,
      double driveGearRatio,
      double angleKP,
      double angleKI,
      double angleKD,
      InvertedWrapper driveMotorInvert,
      InvertedWrapper angleMotorInvert,
      SensorDirectionWrapper cancoderInvert) {
    this.wheelDiameter = wheelDiameter * Math.PI;
    this.wheelCircumference = wheelDiameter;
    this.angleGearRatio = angleGearRatio;
    this.driveGearRatio = driveGearRatio;
    this.angleKP = angleKP;
    this.angleKI = angleKI;
    this.angleKD = angleKD;
    this.driveMotorInvert = driveMotorInvert;
    this.angleMotorInvert = angleMotorInvert;
    this.absoluteEncoderInvert = cancoderInvert;
  }

  public CotsModuleObject withConfiguration(
      String manufacturer, String moduleType, String MotorType, String level) throws Exception {
    File deployDirectory = Filesystem.getDeployDirectory();
    assert deployDirectory.exists();
    File directory = new File(deployDirectory, "configs/swerve/modules");
    assert directory.exists();
    File manufacturer_directory = new File(directory, manufacturer);
    assert manufacturer_directory.exists();
    String name = moduleType + ".json";
    File moduleFile = new File(manufacturer_directory, name);
    assert new File(manufacturer_directory, name).exists();
    LoadedConfig cfg = new LoadedConfig();
    CotsModuleObject cots;
    try {
      cfg = new ObjectMapper().readValue(moduleFile, LoadedConfig.class);
      driveGearRatio = 0.00;
      switch (level) {
        case "L1":
          driveGearRatio = cfg.driveGearRatios.L1;
          break;
        case "L2":
          driveGearRatio = cfg.driveGearRatios.L2;
          break;
        case "L3":
          driveGearRatio = cfg.driveGearRatios.L3;
          break;
        case "L4":
          driveGearRatio = cfg.driveGearRatios.L4;
          break;
        case "X1_10":
          driveGearRatio = cfg.driveGearRatios.X1_10;
          break;
        case "X1_11":
          driveGearRatio = cfg.driveGearRatios.X1_11;
          break;
        case "X1_12":
          driveGearRatio = cfg.driveGearRatios.X1_12;
          break;
        case "X2_10":
          driveGearRatio = cfg.driveGearRatios.X2_10;
          break;
        case "X2_11":
          driveGearRatio = cfg.driveGearRatios.X2_11;
          break;
        case "X2_12":
          driveGearRatio = cfg.driveGearRatios.X2_12;
          break;
        case "X3_10":
          driveGearRatio = cfg.driveGearRatios.X3_12;
          break;
        case "X3_11":
          driveGearRatio = cfg.driveGearRatios.X3_13;
          break;
        case "X3_12":
          driveGearRatio = cfg.driveGearRatios.X3_14;
          break;
        default:
          throw new Exception("Module Config - driveGearRatio not found");
      }

      switch (MotorType) {
        case "Falcon500":
          cots =
              new CotsModuleObject(
                  cfg.Falcon500.wheelDiameter,
                  cfg.Falcon500.angleGearRatio,
                  driveGearRatio,
                  cfg.Falcon500.angleKP,
                  cfg.Falcon500.angleKI,
                  cfg.Falcon500.angleKD,
                  new InvertedWrapper(cfg.Falcon500.driveMotorInvert),
                  new InvertedWrapper(cfg.Falcon500.angleMotorInvert),
                  new SensorDirectionWrapper(cfg.Falcon500.absoluteEncoderInvert));
          break;
        case "KrakenX60":
          cots =
              new CotsModuleObject(
                  cfg.KrakenX60.wheelDiameter,
                  cfg.KrakenX60.angleGearRatio,
                  driveGearRatio,
                  cfg.KrakenX60.angleKP,
                  cfg.KrakenX60.angleKI,
                  cfg.KrakenX60.angleKD,
                  new InvertedWrapper(cfg.KrakenX60.driveMotorInvert),
                  new InvertedWrapper(cfg.KrakenX60.angleMotorInvert),
                  new SensorDirectionWrapper(cfg.KrakenX60.absoluteEncoderInvert));
          break;
        case "Neo":
          cots =
              new CotsModuleObject(
                  cfg.Neo.wheelDiameter,
                  cfg.Neo.angleGearRatio,
                  driveGearRatio,
                  cfg.Neo.angleKP,
                  cfg.Neo.angleKI,
                  cfg.Neo.angleKD,
                  new InvertedWrapper(cfg.Neo.driveMotorInvert),
                  new InvertedWrapper(cfg.Neo.angleMotorInvert),
                  new SensorDirectionWrapper(cfg.Neo.absoluteEncoderInvert));
          break;
        case "Vortex":
          cots =
              new CotsModuleObject(
                  cfg.Vortex.wheelDiameter,
                  cfg.Vortex.angleGearRatio,
                  driveGearRatio,
                  cfg.Vortex.angleKP,
                  cfg.Vortex.angleKI,
                  cfg.Vortex.angleKD,
                  new InvertedWrapper(cfg.Vortex.driveMotorInvert),
                  new InvertedWrapper(cfg.Vortex.angleMotorInvert),
                  new SensorDirectionWrapper(cfg.Vortex.absoluteEncoderInvert));
          break;
        default:
          throw new Exception("Module Config not found");
      }
    } catch (IOException e) {
      e.printStackTrace();
      throw new Exception("Module Unable to be Configured");
    }
    return cots;
  }

  public CotsModuleSwerveConstants to() {
    return new CotsModuleSwerveConstants(
        wheelDiameter,
        angleGearRatio,
        driveGearRatio,
        angleKP,
        angleKI,
        angleKD,
        driveMotorInvert,
        angleMotorInvert,
        absoluteEncoderInvert);
  }
}
