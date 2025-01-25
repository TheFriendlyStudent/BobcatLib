package BobcatLib.Hardware.Sensors.SpatialSensor;

import edu.wpi.first.math.geometry.Rotation2d;
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
  public default void updateInputs(SpatialIOInputs inputs, Rotation2d angle, boolean isEnabled) {}
}
