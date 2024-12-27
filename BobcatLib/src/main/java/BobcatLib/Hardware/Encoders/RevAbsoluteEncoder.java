package BobcatLib.Hardware.Encoders;

import BobcatLib.Utilities.CANDeviceDetails;
import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkAnalogSensor;
import com.revrobotics.SparkAnalogSensor.Mode;

/** Rev absolute encoder, attached through the data port analog pin. */
public class RevAbsoluteEncoder {
  public SparkAnalogSensor encoder;
  public boolean isInverted = false;
  public EncoderConstants chosenModule;
  private CANDeviceDetails details;

  public RevAbsoluteEncoder(
      CANDeviceDetails details, CANSparkMax motor, EncoderConstants chosenModule) {
    this.details = details;
    this.chosenModule = chosenModule;
    configAbsEncoder();
    encoder = motor.getAnalog(Mode.kAbsolute);
  }

  public RevAbsoluteEncoder(
      CANDeviceDetails details, CANSparkFlex motor, EncoderConstants chosenModule) {
    this.details = details;
    this.chosenModule = chosenModule;
    configAbsEncoder();
    encoder = motor.getAnalog(Mode.kAbsolute);
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
    return encoder.getPosition();
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
