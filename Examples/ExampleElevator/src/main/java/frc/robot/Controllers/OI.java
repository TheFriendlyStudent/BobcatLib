package frc.robot.Controllers;

import com.fasterxml.jackson.databind.ObjectMapper;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Controllers.parser.ControllerJson;

import java.io.File;
import java.io.IOException;

/**
 * The Operator Interface (OI) class manages the controller inputs and button
 * mappings for the
 * robot's driver. It supports various controller types and their
 * configurations.
 */
public class OI {
  /** The driver joystick. */
  public final Joystick driver;

  /** Driver raiseToNext */
  public final Trigger raiseToNext;

  /** Driver lowerToPrev */
  public final Trigger lowerToPrev;

  public final Trigger extend;

  /* controller configuration */
  public ControllerJson controllerJson;

  /**
   * The wrapper for the driver controller, supporting different controller types.
   */
  public ControllerWrapper controller;


  /**
   * Constructs the Operator Interface (OI) with the specified driver port and
   * configuration.
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
    raiseToNext = controller.getAorCross();
    lowerToPrev = controller.getBorCircle();
    extend = controller.getYorTriangle();
  }

  public void init(String type, int driverPort) {
    switch (type) {
      case "xbox":
        controller = new XboxControllerWrapper(driverPort);
        break;
      case "ps4":
        controller = new PS4ControllerWrapper(driverPort);
      case "ps5":
        controller = new PS5ControllerWrapper(driverPort);
      case "ruffy":
        controller = new Ruffy(driverPort);
      case "logitech":
        controller = new Logitech(driverPort);
      case "eightbitdo":
        controller = new EightBitDo(driverPort);
      default:
        controller = new XboxControllerWrapper(driverPort);
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
    File directory = new File(deployDirectory, "configs/Elevator");
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
    return controller.getTranslationAxis();
  }

  /**
   * Gets Driver Strafe Axis
   *
   * @return double
   */
  public double getStrafeValue() {
    return controller.getStrafeAxis();
  }

  /**
   * Gets Driver Rotation Axis
   *
   * @return double
   */
  public double getRotationValue() {
    return controller.getRotationAxis();
  }
}
