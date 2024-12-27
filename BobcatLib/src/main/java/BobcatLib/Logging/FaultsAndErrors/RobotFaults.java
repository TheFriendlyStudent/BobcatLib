package BobcatLib.Logging.FaultsAndErrors;

import BobcatLib.Logging.Alert;
import BobcatLib.Logging.Alert.AlertType;
import java.util.ArrayList;
import java.util.List;

public class RobotFaults implements FaultsWrapper {

  public List<Alert> alerts = new ArrayList<Alert>();

  public RobotFaults() {}

  public void createAlert(String msg, AlertType level) {
    Alert alert = new Alert("Robot", msg, level);
    alerts.add(alert);
  }

  public void activateAlert(Alert alert) {
    alert.set(true);
    alert.logAlert("Robot");
  }

  public void activateAlert(Alert alert, AlertType type) {
    alert.setLevel(type);
    alert.set(true);
  }

  public void disableAlert(Alert alert) {
    alert.set(false);
  }
}
