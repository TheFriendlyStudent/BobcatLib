package BobcatLib.Hardware.Encoders;

import edu.wpi.first.math.geometry.Rotation2d;

public interface EncoderIO {
  /** Represents the inputs for the gyro sensor. */
  public static class EncoderIOInputs {
    /** The current position of the encoder. */
    public Rotation2d getEncoderPosition = new Rotation2d();
    /** The current roll position of the encoder. */
    public boolean faulted = false;
  }

  /**
   * Updates the encoder inputs based on external sources.
   *
   * @param inputs The inputs to update.
   */
  public default void updateInputs(EncoderIOInputs inputs) {}

  public default void periodic(CanCoderWrapper encoder) {}

  public default void periodic(ThriftyAbsoluteEncoder encoder) {}

  public default void periodic() {}

  /** Configs the absolute encoder sensor position , determining invertion. */
  public default void configAbsEncoder() {}

  /**
   * Get the absolute position of the encoder.
   *
   * @return Absolute position in rotation (-1 to 0 ) or (0 to 1)
   */
  public default double getAbsolutePosition() {
    return 0.0;
  }

  /**
   * Reset the encoder to factory defaults.
   *
   * <p>This will not clear the stored zero offset.
   */
  public default void factoryDefault() {}

  /** Clear sticky faults on the encoder. */
  public default void clearStickyFaults() {}

  public default void checkForFaults() {}
}
