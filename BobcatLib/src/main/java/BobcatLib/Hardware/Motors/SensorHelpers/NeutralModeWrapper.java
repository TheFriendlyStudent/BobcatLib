package BobcatLib.Hardware.Motors.SensorHelpers;

import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.CANSparkBase.IdleMode;

/** The state of the motor controller bridge when output is neutral or disabled. */
public class NeutralModeWrapper {
  /** current mode. */
  disabledMode mode = disabledMode.None;

  public enum disabledMode {
    Coast,
    Brake,
    None
  }

  /**
   * Constructor
   *
   * @param mode2 value as neutralMode
   */
  public NeutralModeWrapper(NeutralModeValue mode2) {
    mode = disabledMode.Coast;
    if (mode2 == NeutralModeValue.Brake) {
      mode = disabledMode.Brake;
    }
  }

  /**
   * Constructor
   *
   * @param value value as IdleMode
   */
  public NeutralModeWrapper(IdleMode value) {
    mode = disabledMode.Coast;
    if (value == IdleMode.kBrake) {
      mode = disabledMode.Brake;
    }
  }
  /**
   * Constructor
   *
   * @param value value as IdleMode
   */
  public NeutralModeWrapper(disabledMode value) {
    mode = value;
  }

  /**
   * @return
   */
  public IdleMode asIdleMode() {
    if (mode == disabledMode.Brake) {
      return IdleMode.kBrake;
    }
    return IdleMode.kCoast;
  }

  /**
   * As neutralMode Value
   *
   * @return NeutralModeValue
   */
  public NeutralModeValue asNeutralModeValue() {
    if (mode == disabledMode.Brake) {
      return NeutralModeValue.Brake;
    }
    return NeutralModeValue.Coast;
  }
}
