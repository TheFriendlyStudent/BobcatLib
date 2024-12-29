package BobcatLib.Hardware.Motors.SensorHelpers;

import com.ctre.phoenix6.signals.SensorDirectionValue;

/** Sensor Direction Wrapper */
public class SensorDirectionWrapper {
  /** internal state */
  public boolean isInverted = false;

  /**
   * Constructor
   *
   * @param mode
   */
  public SensorDirectionWrapper(boolean mode) {
    isInverted = mode;
  }

  /**
   * Contructor
   *
   * @param mode SensorDirectionValue
   */
  public SensorDirectionWrapper(SensorDirectionValue mode) {
    isInverted = false;
    if (mode == SensorDirectionValue.Clockwise_Positive) {
      isInverted = true;
    }
  }

  /**
   * Returns sensor direction.
   *
   * @return SensorDirectionValue
   */
  public SensorDirectionValue asSensorDirectionValue() {
    if (isInverted) {
      return SensorDirectionValue.Clockwise_Positive;
    }
    return SensorDirectionValue.CounterClockwise_Positive;
  }

  /**
   * gets sensor direction result as a boolean.
   *
   * @return boolean
   */
  public boolean asBoolean() {
    return isInverted;
  }
}
