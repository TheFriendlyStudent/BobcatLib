package BobcatLib.Hardware.Gyros;

import BobcatLib.Hardware.Gyros.parser.GyroDeviceJson;
import BobcatLib.Hardware.Gyros.parser.GyroJson;
import BobcatLib.Logging.FaultsAndErrors.FaultsWrapper;
import BobcatLib.Logging.FaultsAndErrors.Pigeon2Faults;
import BobcatLib.Utilities.CANDeviceDetails;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.Pigeon2Configuration;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.fasterxml.jackson.databind.ObjectMapper;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.StringSubscriber;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.Filesystem;
import java.io.File;
import java.io.IOException;

/**
 * Represents a Pigeon2 gyro sensor, providing methods to interact with the gyro hardware. It can
 * load its configuration from a file, apply settings, and handle faults.
 */
public class Pigeon2Gyro implements GyroIO {
  private FaultsWrapper gyroFaults;
  private Pigeon2 gyro;

  // Subscribed to the configuration changes
  public StringSubscriber configSubscriber;
  public String lastConfig = "{}";
  public GyroJson jsonGyro = new GyroJson();

  /**
   * Constructs a Pigeon2Gyro instance with the specified CAN device details and enables the gyro.
   *
   * @param details The details of the CAN device, including the device number and CAN bus.
   * @param enableGyro Whether to enable the gyro functionality.
   */
  public Pigeon2Gyro(CANDeviceDetails details, boolean enableGyro) {
    jsonGyro.imu = new GyroDeviceJson(details.getDeviceNumber(), details.getBus());
    jsonGyro.enable = enableGyro;
    configGyro();
  }

  /**
   * Default constructor that loads the gyro configuration from a file and applies the
   * configuration.
   */
  public Pigeon2Gyro() {
    loadConfigurationFromFile();
    configGyro();
  }

  /**
   * Configures the gyro sensor by applying a Pigeon2 configuration, resetting it, and setting the
   * yaw to zero.
   */
  public void configGyro() {
    gyro = new Pigeon2(jsonGyro.imu.getId(), jsonGyro.imu.getCanbus());
    gyro.getConfigurator().apply(new Pigeon2Configuration());
    gyro.reset();
    gyro.setYaw(0);
    gyroFaults = new Pigeon2Faults(gyro, gyro.getDeviceID());
  }

  /**
   * Loads the gyro configuration from a JSON file stored in the robot's deploy directory.
   *
   * @return The loaded GyroJson object containing the gyro configuration.
   */
  public GyroJson loadConfigurationFromFile() {
    File deployDirectory = Filesystem.getDeployDirectory();
    assert deployDirectory.exists();
    File directory = new File(deployDirectory, "configs/swerve");
    assert new File(directory, "gyro.json").exists();
    File gyroFile = new File(directory, "gyro.json");
    assert gyroFile.exists();

    try {
      jsonGyro = new ObjectMapper().readValue(gyroFile, GyroJson.class);
    } catch (IOException e) {
      jsonGyro.imu = new GyroDeviceJson(35, "");
    }
    return jsonGyro;
  }

  /**
   * Periodic method to be called during each cycle, can be extended to implement periodic updates.
   */
  public void periodic() {}

  /**
   * Updates the gyro inputs based on the current gyro data.
   *
   * @param inputs The inputs to be updated with the current yaw and fault status.
   */
  public void updateInputs(GyroIOInputs inputs) {
    inputs.yawPosition = Rotation2d.fromDegrees(getYaw());
    inputs.faulted = gyroFaults.hasFaultOccured();
    inputs.rollPosition = Rotation2d.fromDegrees(getRoll());
    inputs.pitchPosition = Rotation2d.fromDegrees(getPitch());
  }

  /**
   * Sets the yaw value of the gyro sensor.
   *
   * @param yaw The yaw value to set, in degrees.
   */
  public void setYaw(double yaw) {
    gyro.setYaw(yaw);
  }

  /**
   * Gets the current yaw value of the gyro sensor.
   *
   * @return The current yaw value, in degrees.
   */
  private double getYaw() {
    StatusSignal<Angle> raw_yaw = gyro.getYaw();
    return raw_yaw.getValueAsDouble();
  }

  /**
   * Sets the roll value of the gyro sensor. Currently, this method is a placeholder and does not
   * modify the roll value.
   *
   * @param roll The roll value to set, in degrees.
   */
  public void setRoll(double roll) {}

  /**
   * Gets the current roll value of the gyro sensor.
   *
   * @return The current roll value, in degrees.
   */
  private double getRoll() {
    StatusSignal<Angle> raw_roll = gyro.getRoll();
    return raw_roll.getValueAsDouble();
  }

  /**
   * Sets the pitch value of the gyro sensor. Currently, this method is a placeholder and does not
   * modify the pitch value.
   *
   * @param pitch The pitch value to set, in degrees.
   */
  public void setPitch(double pitch) {}

  /**
   * Gets the current pitch value of the gyro sensor.
   *
   * @return The current pitch value, in degrees.
   */
  private double getPitch() {
    StatusSignal<Angle> raw_pitch = gyro.getPitch();
    return raw_pitch.getValueAsDouble();
  }

  /**
   * Gets the time difference since the last update.
   *
   * @return The time difference, currently returning a fixed value of 1.0 second.
   */
  public double getTimeDiff() {
    return 1.0;
  }
}
