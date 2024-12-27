package BobcatLib.Hardware.LedControllers;

import BobcatLib.Hardware.LedControllers.CANdleWrapper.CANdleState;

public interface LedControllerIO {
  public class LedControllerIOInputs {
    public CANdleState currState = CANdleState.OFF;
  }
  /**
   * update the inputs for IO layer
   *
   * @param inputs CANdleIOInputs
   */
  public default void updateInputs(LedControllerIOInputs inputs) {}

  /**
   * THIS WILL NOT AUTOMATICALLY TURN OFF, it will persist untill you set it again
   *
   * @param state the animation to play
   */
  public default void setLEDs(CANdleState state) {}

  /**
   * @param state the animation to play
   * @param seconds duration to play
   */
  public default void setLEDs(CANdleState state, double seconds) {}

  /** Periodic for CANdle hardware */
  public default void periodic() {}

  public default void checkForFaults() {}
}
