package BobcatLib.Hardware.Sensors.SpatialSensor.Components;

import BobcatLib.Hardware.Sensors.SpatialSensor.Utility.DistanceMode;
import BobcatLib.Logging.Alert;
import BobcatLib.Logging.Alert.AlertType;
import com.ctre.phoenix6.configs.CANrangeConfiguration;
import com.ctre.phoenix6.hardware.CANrange;

public class CANRange implements RangeSensor {
  public int id;
  public CANrange tof;
  public final double sampleTime;
  public DistanceMode mode;
  public double range;
  public Alert sensorAlert;
  public boolean enable = false;

  public CANRange(int id, DistanceMode mode, double sampleTime, String busname) {
    this.id = id;
    this.sampleTime = sampleTime;
    this.mode = mode;
    try {
      if (busname == "" || busname == "rio") {
        // Construct the CANrange
        tof = new CANrange(id);
      } else {
        // Construct the CANrange
        tof = new CANrange(id, busname);
      }

      configRangeSensor();
      enable = true;
    } catch (Exception e) {
      // TODO: handle exception
      AlertType level = AlertType.INFO;
      sensorAlert = new Alert("TOF", "TOF " + id + " hardware fault occured", level);
      sensorAlert.set(true);
    }
  }

  public CANRange(int id, DistanceMode mode, double sampleTime) {
    this.id = id;
    this.sampleTime = sampleTime;
    this.mode = mode;
    try {

      // Construct the CANrange
      tof = new CANrange(id);

      configRangeSensor();
      enable = true;
    } catch (Exception e) {
      // TODO: handle exception
      AlertType level = AlertType.INFO;
      sensorAlert = new Alert("TOF", "TOF " + id + " hardware fault occured", level);
      sensorAlert.set(true);
    }
  }

  /**
   * Gets the range in front of the sensor.
   *
   * @return range in mm
   */
  public double getRange() {
    if (!enable) {
      return 0;
    }
    // Get Distance
    range = tof.getDistance().getValueAsDouble();

    return range;
  }

  public void configRangeSensor() {
    // Configure the CANrange for basic use
    CANrangeConfiguration configs = new CANrangeConfiguration();

    // Write these configs to the CANrange
    tof.getConfigurator().apply(configs);
  }

  public void configRangeSensor(DistanceMode m) {
    this.mode = m;
  }

  public DistanceMode getMode() {
    return mode;
  }

  public DistanceMode getOptimalMode() {
    double distance = getRange();
    DistanceMode distanceMode = new DistanceMode();
    distanceMode.currentMode = DistanceMode.modes.MEDIUM;
    return distanceMode;
  }
}
