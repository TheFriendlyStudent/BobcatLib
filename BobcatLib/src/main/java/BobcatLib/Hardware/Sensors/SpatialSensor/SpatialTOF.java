package BobcatLib.Hardware.Sensors.SpatialSensor;

import BobcatLib.Hardware.Sensors.SpatialSensor.Components.RangeSensor;
import BobcatLib.Hardware.Sensors.SpatialSensor.Utility.DistanceMode;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;

public class SpatialTOF implements SpatialIO {
  /** Sensors as identified by "field centric" side. */
  public HashMap<String, RangeSensor[]> mRangeSensors;

  public SpatialTOF(HashMap<String, RangeSensor[]> sensors) {
    mRangeSensors = sensors;
    configAllSensors();
  }

  /**
   * Updates the gyro inputs based on external sources.
   *
   * @param inputs The inputs to update.
   */
  public void updateInputs(SpatialIOInputs inputs) {
    HashMap<String, Double> distances = detectObjects();
    inputs.front_left_distance = distances.get("left");
    inputs.front_right_distance = distances.get("right");
  }

  /** Configure all Sensors */
  public void configAllSensors() {
    mRangeSensors.put("left", new RangeSensor[] {});
    mRangeSensors.put("right", new RangeSensor[] {});
  }

  /**
   * Find the object distances on all sides.
   *
   * @return range in mm
   */
  public HashMap<String, Double> detectObjects() {
    HashMap<String, Double> mDistances = new HashMap<String, Double>();
    for (String label : new String[] {"left", "right"}) {
      double avg = detectObject(mRangeSensors.get(label));
      mDistances.put(label, avg);
    }

    return mDistances;
  }

  /**
   * Find the object distance. If the sensor is in long mode and calculated distance is less that
   * 1250mm switch to short mode otherwise keep in long mode. If in short keep in short if less than
   * 1250mm otherwise switch back to long range mode
   *
   * @return range in mm
   */
  public Double detectObject(RangeSensor[] sensors) {
    if (sensors.length < 1) {
      return 0.00;
    }
    ArrayList<Double> distances = new ArrayList<Double>(2);
    for (RangeSensor sensor : sensors) {
      double range = sensor.getRange();
      setCustomRangingMode(sensor);
      distances.add(range);
    }
    double avg = calculateAverage(distances);
    return avg;
  }

  public void setCustomRangingMode(RangeSensor sensor) {
    DistanceMode mode = sensor.getOptimalMode();
    sensor.configRangeSensor(mode);
  }

  /**
   * calculates the average distances of an arraylist of double
   *
   * @param sensorDistances
   * @return average distance in mm
   */
  private double calculateAverage(List<Double> sensorDistances) {
    return sensorDistances.stream().mapToDouble(d -> d).average().orElse(0.0);
  }

  public HashMap<String, RangeSensor[]> getRangeSensors() {
    return mRangeSensors;
  }
}
