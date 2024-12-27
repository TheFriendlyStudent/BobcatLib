package BobcatLib.Hardware.Encoders;

import org.littletonrobotics.junction.Logger;

public class BaseEncoder {
  private final EncoderIO io;
  private final EncoderIOInputsAutoLogged inputs = new EncoderIOInputsAutoLogged();

  public BaseEncoder(EncoderIO io) {
    this.io = io;
  }

  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("encoder", inputs);
  }

  /** Configs the absolute encoder sensor position , determining invertion. */
  public void configAbsEncoder() {
    io.configAbsEncoder();
    checkForFaults();
  }

  /**
   * Get the absolute position of the encoder.
   *
   * @return Absolute position in rotation (-1 to 0 ) or (0 to 1)
   */
  public double getAbsolutePosition() {
    double absPos = io.getAbsolutePosition();
    checkForFaults();
    return absPos;
  }

  /**
   * Reset the encoder to factory defaults.
   *
   * <p>This will not clear the stored zero offset.
   */
  public void factoryDefault() {
    io.factoryDefault();
    checkForFaults();
  }

  /** Clear sticky faults on the encoder. */
  public void clearStickyFaults() {
    io.clearStickyFaults();
    checkForFaults();
  }

  public void checkForFaults() {
    io.checkForFaults();
  }
}
