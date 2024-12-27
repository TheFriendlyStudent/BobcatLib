package BobcatLib.Logging.FaultsAndErrors;

import BobcatLib.Logging.Alert;
import BobcatLib.Logging.Alert.AlertType;

public class SparkMaxFaults implements FaultsWrapper {
  public enum faults {
    Brownout,
    Overcurrent,
    MotorFault,
    SensorFault,
    Stall,
    kHasReset
  }

  public static Alert BrownoutAlert;
  public static Alert OvercurrentAlert;
  public static Alert MotorFaultAlert;
  public static Alert SensorFaultAlert;
  public static Alert StallAlert;
  public static Alert kHasResetAlert;
  public int id;

  public SparkMaxFaults(int id) {
    AlertType level = AlertType.INFO;
    BrownoutAlert = new Alert("SparkMax", "SparkMax " + id + " hardware fault occured", level);
    OvercurrentAlert = new Alert("SparkMax", "SparkMax " + id + " has overcurrent", level);
    MotorFaultAlert = new Alert("SparkMax", "SparkMax " + id + " has motor fault", level);
    SensorFaultAlert = new Alert("SparkMax", "SparkMax " + id + " has sensor fault", level);
    StallAlert = new Alert("SparkMax", "SparkMax " + id + " has stalled", level);
    kHasResetAlert = new Alert("SparkMax", "SparkMax " + id + " has reset", level);
  }

  public void activateAlert(Alert alert) {
    alert.set(true);
    alert.logAlert("SparkMax " + id);
  }

  public void activateAlert(Alert alert, AlertType type) {
    alert.setLevel(type);
    alert.set(true);
  }

  public void disableAlert(Alert alert) {
    alert.set(false);
  }
}
