package BobcatLib.Logging.FaultsAndErrors;

import BobcatLib.Logging.Alert;
import BobcatLib.Logging.Alert.AlertType;
import com.ctre.phoenix.led.CANdle;
import com.ctre.phoenix.led.CANdleFaults;
import java.util.ArrayList;
import java.util.List;
import java.util.Map;
import java.util.function.BooleanSupplier;

public class CANdleFaultsWrapper implements FaultsWrapper {
  public int id;
  public CANdle leds;
  public static Alert hardwareAlert;
  public static Alert underVoltagedAlert, overVoltagedAlert;

  public CANdleFaultsWrapper(CANdle leds, int id) {
    this.id = id;
    this.leds = leds;
    AlertType level = AlertType.INFO;
    overVoltagedAlert =
        new Alert("Leds", "CANdle " + id + " is undervoltaged, potential brownout.", level);

    underVoltagedAlert =
        new Alert("Leds", "CANdle " + id + " is undervoltaged, potential brownout.", level);
    hardwareAlert = new Alert("Leds", "CANdle " + id + " has a hardware fault.", level);
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
    CANdleFaults ledFaults = new CANdleFaults();
    leds.getFaults(ledFaults);
    Map<BooleanSupplier, Alert> faultChecks =
        Map.of(
            () -> ledFaults.V5TooLow, underVoltagedAlert,
            () -> ledFaults.V5TooHigh, overVoltagedAlert,
            () -> ledFaults.HardwareFault, hardwareAlert);

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
