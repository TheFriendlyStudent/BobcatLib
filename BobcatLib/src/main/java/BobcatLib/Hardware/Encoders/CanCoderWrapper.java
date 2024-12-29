package BobcatLib.Hardware.Encoders;

import BobcatLib.Logging.Alert;
import BobcatLib.Logging.FaultsAndErrors.CanCoderFaults;
import BobcatLib.Utilities.CANDeviceDetails;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;

public class CanCoderWrapper implements EncoderIO {
  public CANcoderConfiguration swerveCANcoderConfig = new CANcoderConfiguration();
  private CANcoder encoder;
  /** Last angle reading was faulty. */
  public boolean readingError = false;

  public boolean isInverted = false;
  /** An {@link Alert} for if the CAN ID is greater than 40. */
  public static final Alert canIdWarning =
      new Alert(
          "JSON",
          "CAN IDs greater than 40 can cause undefined behaviour, please use a CAN ID below 40!",
          Alert.AlertType.WARNING);

  public EncoderConstants chosenModule;

  private CanCoderFaults faults;

  private CANDeviceDetails details;

  public CanCoderWrapper(
      CANDeviceDetails details, EncoderConstants chosenModule, String canivorename) {
    this.details = details;
    int id = details.getDeviceNumber();
    if (id >= 40) {
      canIdWarning.set(true);
    }
    this.chosenModule = chosenModule;
    configAbsEncoder();

    /* Angle Encoder Config */
    encoder = new CANcoder(id, canivorename);
    encoder.getConfigurator().apply(swerveCANcoderConfig);

    faults = new CanCoderFaults(encoder, id);
  }

  /** Configs the absolute encoder sensor position , determining invertion. */
  public void configAbsEncoder() {
    /** Swerve CANCoder Configuration */
    swerveCANcoderConfig.MagnetSensor.SensorDirection =
        chosenModule.absoluteEncoderInvert.asSensorDirectionValue();
    isInverted = chosenModule.absoluteEncoderInvert.asBoolean();
  }

  /**
   * Get the absolute position of the encoder.
   *
   * @return Absolute position in rotation (-0.5 to 0.999999 )
   */
  public double getAbsolutePosition() {
    return encoder.getAbsolutePosition().getValueAsDouble();
  }

  /** Reset the encoder to factory defaults. */
  public void factoryDefault() {
    encoder.getConfigurator().apply(new CANcoderConfiguration());
  }

  /** Clear sticky faults on the encoder. */
  public void clearStickyFaults() {
    encoder.clearStickyFaults();
  }

  public void checkForFaults() {
    faults.hasFaultOccured();
  }
}
