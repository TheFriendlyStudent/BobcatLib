package BobcatLib.Hardware.Controllers;

import BobcatLib.Hardware.Controllers.parser.ControllerJson;
import com.fasterxml.jackson.databind.ObjectMapper;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import java.io.File;
import java.io.IOException;

/**
 * The Operator Interface (OI) class manages the controller inputs and button mappings for the
 * robot's driver. It supports various controller types and their configurations.
 */
public class OI {
  /** The driver joystick. */
  public final Joystick driver;

  /** Driver Buttons */
  public final Trigger robotCentric;

  /** Driver Zero Gyros */
  public final Trigger zeroGyro;

  /** Spatial Trigger */
  public final Trigger spatialTrigger;

  /* controller configuration */
  public ControllerJson controllerJson;

  /** The wrapper for the driver controller, supporting different controller types. */
  public ControllerWrapper driver_controller;

  /** Trigger for the D-Pad forward button. */
  public Trigger dpadForwardBtn;

  /** Trigger for the D-Pad backward button. */
  public Trigger dpadBackBtn;

  /** Trigger for the D-Pad left button. */
  public Trigger dpadLeftBtn;

  /** Trigger for the D-Pad right button. */
  public Trigger dpadRightBtn;

  /**
   * Constructs the Operator Interface (OI) with the specified driver port and configuration.
   * Initializes controller inputs and button mappings.
   */
  public OI() {
    loadConfigurationFromFile();
    int driverPort = controllerJson.driver.id;
    String type = controllerJson.driver.type;
    /* USB Xbox Controllers */
    driver = new Joystick(driverPort);
    /* Driver Buttons */
    init(type, driverPort);
    robotCentric = driver_controller.getLeftBumper();
    zeroGyro = driver_controller.getRightBumper();
    spatialTrigger = driver_controller.getLeftTrigger();
    dpadForwardBtn = driver_controller.getDPadTriggerUp();
    dpadBackBtn = driver_controller.getDPadTriggerDown();
    dpadLeftBtn = driver_controller.getDPadTriggerLeft();
    dpadRightBtn = driver_controller.getDPadTriggerRight();
  }

  public void init(String type, int driverPort) {
    switch (type) {
      case "xbox":
        driver_controller = new XboxControllerWrapper(driverPort);
        break;
      case "ps4":
        driver_controller = new PS4ControllerWrapper(driverPort);
      case "ps5":
        driver_controller = new PS5ControllerWrapper(driverPort);
      case "ruffy":
        driver_controller = new Ruffy(driverPort);
      case "logitech":
        driver_controller = new Logitech(driverPort);
      case "eightbitdo":
        driver_controller = new EightBitDo(driverPort);
      default:
        driver_controller = new XboxControllerWrapper(driverPort);
    }
  }

  /**
   * Loads Configuration From Deployed File
   *
   * @return ControllerJson
   */
  public ControllerJson loadConfigurationFromFile() {
    String name = "oi.json";
    File deployDirectory = Filesystem.getDeployDirectory();
    assert deployDirectory.exists();
    File directory = new File(deployDirectory, "configs/swerve");
    assert new File(directory, name).exists();
    File moduleFile = new File(directory, name);
    assert moduleFile.exists();
    controllerJson = new ControllerJson();
    try {
      controllerJson = new ObjectMapper().readValue(moduleFile, ControllerJson.class);
    } catch (IOException e) {

    }
    return controllerJson;
  }

  /**
   * Gets Driver Translation Axis
   *
   * @return double
   */
  public double getTranslationValue() {
    return driver_controller.getTranslationAxis();
  }

  /**
   * Gets Driver Strafe Axis
   *
   * @return double
   */
  public double getStrafeValue() {
    return driver_controller.getStrafeAxis();
  }

  /**
   * Gets Driver Rotation Axis
   *
   * @return double
   */
  public double getRotationValue() {
    return driver_controller.getRotationAxis();
  }
}
