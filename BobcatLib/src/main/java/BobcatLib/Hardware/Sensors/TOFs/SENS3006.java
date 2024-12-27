package BobcatLib.Hardware.Sensors.TOFs;

import BobcatLib.Hardware.Sensors.TOFs.Utility.RangingMode;
import BobcatLib.Logging.Alert;
import BobcatLib.Logging.Alert.AlertType;
import BobcatLib.Utilities.CANDeviceDetails;
import com.playingwithfusion.TimeOfFlight;

public class SENS3006 implements TimeOfFlightIO {
  public int id;
  public TimeOfFlight tof;
  public final double sampleTime;
  public RangingMode mode;
  public double range;
  public Alert sensorAlert;
  public boolean enable = false;
  private CANDeviceDetails details;

  public SENS3006(CANDeviceDetails details, RangingMode mode, double sampleTime) {
    this.details = details;
    int id = details.getDeviceNumber();
    this.sampleTime = sampleTime;
    this.mode = mode;
    try {
      tof = new TimeOfFlight(id);
      configRangeSensor();
      enable = true;
    } catch (Exception e) {
      AlertType level = AlertType.INFO;
      sensorAlert = new Alert("TOF", "TOF " + id + " hardware fault occured", level);
      sensorAlert.set(true);
    }
  }

  public void updateInputs(TimeOfFlightIOInputs inputs) {
    inputs.distance = getRange();
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
    com.playingwithfusion.TimeOfFlight.RangingMode tof_mode;
    if (mode == RangingMode.close) {
      tof_mode = com.playingwithfusion.TimeOfFlight.RangingMode.Long;
    } else if (mode == RangingMode.mid) {
      tof_mode = com.playingwithfusion.TimeOfFlight.RangingMode.Medium;
    } else {
      tof_mode = com.playingwithfusion.TimeOfFlight.RangingMode.Long;
    }
    tof.setRangingMode(tof_mode, sampleTime);
  }

  public void configRangeSensor(RangingMode m) {
    this.mode = m;
    com.playingwithfusion.TimeOfFlight.RangingMode tof_mode;
    if (mode == RangingMode.close) {
      tof_mode = com.playingwithfusion.TimeOfFlight.RangingMode.Long;
    } else if (mode == RangingMode.mid) {
      tof_mode = com.playingwithfusion.TimeOfFlight.RangingMode.Medium;
    } else {
      tof_mode = com.playingwithfusion.TimeOfFlight.RangingMode.Long;
    }
    tof.setRangingMode(tof_mode, sampleTime);
  }

  public RangingMode getMode() {
    return mode;
  }
}
