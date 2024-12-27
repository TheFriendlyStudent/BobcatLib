package BobcatLib.Hardware.Encoders;

import BobcatLib.Utilities.CANDeviceDetails;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.RobotController;

public class ThriftyAbsoluteEncoder implements EncoderIO {
  public AnalogInput encoder;
  private boolean isInverted;
  public EncoderConstants chosenModule;
  private CANDeviceDetails details;

  public ThriftyAbsoluteEncoder(CANDeviceDetails details, EncoderConstants chosenModule) {
    this.details = details;
    this.chosenModule = chosenModule;
    configAbsEncoder();
    encoder = new AnalogInput(details.getDeviceNumber());
  }

  /** Configs the absolute encoder sensor position , determining invertion. */
  public void configAbsEncoder() {
    isInverted = chosenModule.absoluteEncoderInvert.asBoolean();
  }

  /**
   * Get the absolute position of the encoder.
   *
   * @return Absolute position in rotation (-1 to 0 ) or (0 to 1)
   */
  public double getAbsolutePosition() {
    return (isInverted ? -1.0 : 1.0)
        * (encoder.getAverageVoltage() / RobotController.getVoltage5V());
  }

  /** Reset the encoder to factory defaults. */
  public void factoryDefault() {
    // Do nothing
  }

  /** Clear sticky faults on the encoder. */
  public void clearStickyFaults() {
    // Do nothing
  }

  public void checkForFaults() {}
}
