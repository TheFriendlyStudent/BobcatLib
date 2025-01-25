package BobcatLib.Hardware.Sensors.SpatialSensor;

import BobcatLib.Hardware.Sensors.SpatialSensor.Components.RangeSensor;
import java.util.HashMap;
import org.littletonrobotics.junction.AutoLog;

public interface SpatialIO {
  /** Represents the inputs for the Spatial sensor. */
  @AutoLog
  public static class SpatialIOInputs {
    public double front_left_distance = 0.0;
    public double front_right_distance = 0.0;
  }

  /**
   * Updates the gyro inputs based on external sources.
   *
   * @param inputs The inputs to update.
   */
  public default void updateInputs(SpatialIOInputs inputs, boolean isEnabled) {}

  public default HashMap<String, RangeSensor[]> getRangeSensors() {
    return null;
  }
  ;
}
