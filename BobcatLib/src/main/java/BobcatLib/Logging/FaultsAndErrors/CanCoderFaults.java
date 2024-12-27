package BobcatLib.Logging.FaultsAndErrors;

import BobcatLib.Logging.Alert;
import BobcatLib.Logging.Alert.AlertType;
import com.ctre.phoenix6.hardware.CANcoder;
import java.util.ArrayList;
import java.util.List;
import java.util.Map;
import java.util.function.BooleanSupplier;

public class CanCoderFaults implements FaultsWrapper {
  public int id;
  public CANcoder encoder;
  public static Alert underVoltAlert;
  public static Alert magnetOutOfRangeAlert;
  public static Alert outOfTemperatureRangeAlert;
  public static Alert HardwareAlert;
  public static Alert UnlicensedFeatureInUseAlert;
  public static Alert bootDuringEnableAlert;

  public CanCoderFaults(CANcoder enc, int id) {
    this.id = id;
    this.encoder = enc;
    AlertType level = AlertType.INFO;
    underVoltAlert =
        new Alert(
            "Encoders", "CanAndCoder " + id + " is undervoltaged, potential brownout.", level);
    HardwareAlert = new Alert("Encoders", "CanAndCoder " + id + " has a hardware fault.", level);
    UnlicensedFeatureInUseAlert =
        new Alert("Encoders", "CanAndCoder " + id + " is using unlicensed.", level);

    bootDuringEnableAlert =
        new Alert("Encoders", "CanAndCoder " + id + " recently rebooted.", level);
  }

  public void activateAlert(Alert alert) {
    alert.set(true);
    alert.logAlert("CanCoder " + id);
  }

  public void activateAlert(Alert alert, AlertType type) {
    alert.setLevel(type);
    alert.set(true);
  }

  public void disableAlert(Alert alert) {
    alert.set(false);
  }

  public boolean hasFaultOccured() {
    List<Alert> foundFaults = new ArrayList<>();
    Map<BooleanSupplier, Alert> faultChecks =
        Map.of(
            encoder.getFault_Undervoltage()::getValue, underVoltAlert,
            encoder.getFault_BadMagnet()::getValue, magnetOutOfRangeAlert,
            encoder.getFault_Hardware()::getValue, HardwareAlert,
            encoder.getFault_UnlicensedFeatureInUse()::getValue, UnlicensedFeatureInUseAlert,
            encoder.getFault_BootDuringEnable()::getValue, bootDuringEnableAlert);

    faultChecks.forEach(
        (faultCondition, alert) -> {
          if (faultCondition.getAsBoolean()) {
            foundFaults.add(alert);
          }
        });

    foundFaults.forEach(this::activateAlert);

    return !foundFaults.isEmpty();
  }
}
