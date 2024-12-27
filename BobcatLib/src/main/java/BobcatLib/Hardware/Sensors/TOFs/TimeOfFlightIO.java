package BobcatLib.Hardware.Sensors.TOFs;

import BobcatLib.Hardware.Sensors.TOFs.Utility.RangingMode;
import org.littletonrobotics.junction.AutoLog;

public interface TimeOfFlightIO {
  /** Represents the inputs for the gyro sensor. */
  @AutoLog
  public static class TimeOfFlightIOInputs {
    public double distance = 0.00;
  }

  /**
   * Updates the tof inputs based on external sources.
   *
   * @param inputs The inputs to update.
   */
  public default void updateInputs(TimeOfFlightIOInputs inputs) {}

  /**
   * Gets the range in front of the sensor.
   *
   * @return range in mm
   */
  public default double getRange() {
    return 0;
  }

  /** config the sensor with default setting for the device. */
  public default void configRangeSensor() {}

  /**
   * Given the mode , config the sensor to the right detection parameters.
   *
   * @param mode
   */
  public default void configRangeSensor(RangingMode mode) {}

  /**
   * Gets the Ranging Mode of the sensor
   *
   * @return RangingMode
   */
  public default RangingMode getMode() {
    return RangingMode.close;
  }
}
