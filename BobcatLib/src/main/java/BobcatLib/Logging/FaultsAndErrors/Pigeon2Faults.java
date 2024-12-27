package BobcatLib.Logging.FaultsAndErrors;

import BobcatLib.Logging.Alert;
import BobcatLib.Logging.Alert.AlertType;
import com.ctre.phoenix6.hardware.Pigeon2;
import java.util.ArrayList;
import java.util.List;
import java.util.Map;
import java.util.function.BooleanSupplier;

public class Pigeon2Faults implements FaultsWrapper {
  public int id;
  public Pigeon2 imu;
  public static Alert BootDuringEnableAlert;
  public static Alert SaturatedAccelerometerAlert;
  public static Alert SaturatedMagnetometerAlert;
  public static Alert SaturatedGyroScopeAlert;
  public static Alert LoopTimeSlowFaultAlert;
  public static Alert DataAcquiredLateFaultAlert;
  public static Alert BootIntoMotionFaultAlert;
  public static Alert BootupMagnetometerFaultAlert;
  public static Alert BootupGyroscopeFaultAlert;
  public static Alert UndervoltageFaultAlert;
  public static Alert HardwareAlert;

  public Pigeon2Faults(Pigeon2 imu, int id) {
    this.id = id;
    this.imu = imu;
    AlertType level = AlertType.INFO;
    BootDuringEnableAlert =
        new Alert("Gyro", "Pigeon2 " + id + " boot while detecting the enable signal", level);
    SaturatedAccelerometerAlert =
        new Alert("Gyro", "Pigeon2 " + id + " Accelerometer values are saturated", level);
    SaturatedMagnetometerAlert =
        new Alert("Gyro", "Pigeon2 " + id + " Magnetometer values are saturated", level);
    SaturatedGyroScopeAlert =
        new Alert("Gyro", "Pigeon2 " + id + " Gyroscope values are saturated", level);
    LoopTimeSlowFaultAlert =
        new Alert(
            "Gyro", "Pigeon2 " + id + " Motion stack loop time was slower than expected.", level);
    DataAcquiredLateFaultAlert =
        new Alert(
            "Gyro",
            "Pigeon2 " + id + " Motion stack data acquisition was slower than expected",
            level);
    BootIntoMotionFaultAlert =
        new Alert("Gyro", "Pigeon2 " + id + " motion Detected during bootup.", level);
    BootupMagnetometerFaultAlert =
        new Alert("Gyro", "Pigeon2 " + id + " Bootup checks failed: Magnetometer", level);
    BootupGyroscopeFaultAlert =
        new Alert("Gyro", "Pigeon2 " + id + " Bootup checks failed: Gyroscope", level);
    UndervoltageFaultAlert =
        new Alert(
            "Gyro", "Pigeon2 " + id + " supply voltage dropped to near brownout levels", level);
    HardwareAlert = new Alert("Gyro", "Pigeon2 " + id + " Hardware fault occurred", level);
  }

  public void activateAlert(Alert alert) {
    alert.set(true);
    alert.logAlert("Pigeon2 " + id);
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
            imu.getFault_BootDuringEnable()::getValue,
            BootDuringEnableAlert,
            imu.getFault_SaturatedAccelerometer()::getValue,
            SaturatedAccelerometerAlert,
            imu.getFault_SaturatedMagnetometer()::getValue,
            SaturatedMagnetometerAlert,
            imu.getFault_SaturatedGyroscope()::getValue,
            SaturatedGyroScopeAlert,
            imu.getFault_LoopTimeSlow()::getValue,
            LoopTimeSlowFaultAlert,
            imu.getFault_DataAcquiredLate()::getValue,
            DataAcquiredLateFaultAlert,
            imu.getFault_BootIntoMotion()::getValue,
            BootIntoMotionFaultAlert,
            imu.getFault_BootupMagnetometer()::getValue,
            BootupMagnetometerFaultAlert,
            imu.getFault_Undervoltage()::getValue,
            UndervoltageFaultAlert,
            imu.getFault_Hardware()::getValue,
            HardwareAlert);

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
