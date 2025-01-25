package BobcatLib.Hardware.Sensors.SpatialSensor.Components;

import BobcatLib.Hardware.Sensors.SpatialSensor.Utility.DistanceMode;
import BobcatLib.Logging.Alert;
import BobcatLib.Logging.Alert.AlertType;
import com.playingwithfusion.TimeOfFlight;
import com.playingwithfusion.TimeOfFlight.RangingMode;

public class SENS3006 implements RangeSensor {
  public int id;
  public TimeOfFlight tof;
  public final double sampleTime;
  public DistanceMode mode;
  public double range;
  public Alert sensorAlert;
  public boolean enable = false;

  public SENS3006(int id, DistanceMode mode, double sampleTime) {
    this.id = id;
    this.sampleTime = sampleTime;
    this.mode = mode;
    try {
      tof = new TimeOfFlight(id);
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
    range = tof.getRange();
    return range;
  }

  public void configRangeSensor() {
    tof.setRangingMode(mode.asPWF(), sampleTime);
  }

  public void configRangeSensor(DistanceMode m) {
    this.mode = m;
    tof.setRangingMode(mode.asPWF(), sampleTime);
  }

  public DistanceMode getMode() {
    return mode;
  }
  /**
   * Sets the distance mode of the sensor based on the provided distance.
   *
   * <p>RangingMode.Short: if distance less than 1250 mm RangingMode.Medium: if 1250 mm less than or
   * equal too distance less than 2250 mm RangingMode.Long: if distance greater than or equal to
   * 2250 mm
   */
  public DistanceMode getOptimalMode() {
    double distance = getRange();
    RangingMode mode =
        (distance < 1250)
            ? RangingMode.Short
            : (distance < 2250) ? RangingMode.Medium : RangingMode.Long;
    DistanceMode distanceMode = new DistanceMode();
    return distanceMode.fromPWF(mode);
  }
}
