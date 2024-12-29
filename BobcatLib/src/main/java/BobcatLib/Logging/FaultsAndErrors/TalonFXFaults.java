package BobcatLib.Logging.FaultsAndErrors;

import BobcatLib.Logging.Alert;
import BobcatLib.Logging.Alert.AlertType;
import com.ctre.phoenix6.hardware.TalonFX;
import java.util.ArrayList;
import java.util.List;
import java.util.Map;
import java.util.function.BooleanSupplier;

public class TalonFXFaults implements FaultsWrapper {
  public enum faults {
    hardware,
    procTemp,
    deviceTemp,
    underVoltage,
    bootDuringEnable,
    unlicensedFeatureInUse,
    unstableSupplyV
  }

  public static Alert hardwareAlert;
  public static Alert procTempAlert;
  public static Alert deviceTempAlert;
  public static Alert underVoltageAlert;
  public static Alert bootDuringEnableAlert;
  public static Alert unlicensedFeatureInUseAlert;
  public static Alert unstableSupplyVAlert;
  public int id;
  public TalonFX motor;

  public TalonFXFaults(TalonFX motor, int id) {
    this.id = id;
    this.motor = motor;
    AlertType level = AlertType.INFO;
    hardwareAlert = new Alert("Motors", "TalonFX " + id + " hardware fault occured", level);
    procTempAlert =
        new Alert("Motors", "TalonFX " + id + " Processor temperature exceeded limit", level);
    deviceTempAlert =
        new Alert("Motors", "TalonFX " + id + " Device temperature exceeded limit", level);
    underVoltageAlert =
        new Alert(
            "Motors",
            "TalonFX " + id + " Device supply voltage dropped to near brownout levels",
            level);
    bootDuringEnableAlert =
        new Alert(
            "Motors", "TalonFX " + id + " Device boot while detecting the enable signal", level);
    unlicensedFeatureInUseAlert =
        new Alert(
            "Motors",
            "TalonFX "
                + id
                + " An unlicensed feature is in use, device may not behave as expected.",
            level);
    unstableSupplyVAlert =
        new Alert("Motors", "TalonFX " + id + " unstable voltage detected.", level);
  }

  public void activateAlert(Alert alert) {
    alert.set(true);
    alert.logAlert("TalonFX " + id);
  }

  public void activateAlert(Alert alert, AlertType type) {
    alert.setLevel(type);
    alert.set(true);
  }

  public void disableAlert(Alert alert) {
    alert.set(false);
  }

  public boolean detect_fault(boolean faultTest, Alert alert, AlertType alertType) {
    boolean fault = false;
    if (faultTest) {
      fault = true;
    }
    return fault;
  }

  public boolean hasFaultOccured() {
    List<Alert> foundFaults = new ArrayList<>();
    Map<BooleanSupplier, Alert> faultChecks =
        Map.of(
            motor.getFault_ProcTemp()::getValue, procTempAlert,
            motor.getFault_DeviceTemp()::getValue, deviceTempAlert,
            motor.getFault_Undervoltage()::getValue, underVoltageAlert,
            motor.getFault_BootDuringEnable()::getValue, bootDuringEnableAlert,
            motor.getFault_UnlicensedFeatureInUse()::getValue, unlicensedFeatureInUseAlert,
            motor.getFault_UnstableSupplyV()::getValue, unstableSupplyVAlert,
            motor.getFault_Hardware()::getValue, hardwareAlert);
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
