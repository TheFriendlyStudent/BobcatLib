package BobcatLib.Hardware.Sensors.SpatialSensor.Components;

import BobcatLib.Hardware.Sensors.SpatialSensor.Utility.DistanceMode;

public interface RangeSensor {
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
  public default void configRangeSensor(DistanceMode mode) {}

  /**
   * Gets the Ranging Mode of the sensor
   *
   * @return RangingMode
   */
  public default DistanceMode getMode() {
    return null;
  }

  public default DistanceMode getOptimalMode() {
    return null;
  }
}
