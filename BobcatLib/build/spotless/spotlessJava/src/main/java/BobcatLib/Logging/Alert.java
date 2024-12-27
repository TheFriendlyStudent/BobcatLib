package BobcatLib.Logging;

import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import org.littletonrobotics.junction.Logger;

/** Class for managing persistent alerts to be sent over NetworkTables. */
public class Alert {

  /** Group of the alert. */
  private static Map<String, SendableAlerts> groups = new HashMap<String, SendableAlerts>();
  /** Type of the Alert to raise. */
  private AlertType type;
  /** Activation state of alert. */
  private boolean active = false;
  /** When the alert was raised. */
  private double activeStartTime = 0.0;
  /** Text of the alert. */
  private String text;

  /**
   * Creates a new Alert in the default group - "Alerts". If this is the first to be instantiated,
   * the appropriate entries will be added to NetworkTables.
   *
   * @param text Text to be displayed when the alert is active.
   * @param type Alert level specifying urgency.
   */
  public Alert(String text, AlertType type) {
    this("Alerts", text, type);
  }

  /**
   * Creates a new Alert. If this is the first to be instantiated in its group, the appropriate
   * entries will be added to NetworkTables.
   *
   * @param group Group identifier, also used as NetworkTables title
   * @param text Text to be displayed when the alert is active.
   * @param type Alert level specifying urgency.
   */
  public Alert(String group, String text, AlertType type) {
    if (!groups.containsKey(group)) {
      groups.put(group, new SendableAlerts());
      SmartDashboard.putData(group, groups.get(group));
    }

    this.text = text;
    this.type = type;
    groups.get(group).alerts.add(this);
  }

  /**
   * Sets whether the alert should currently be displayed. When activated, the alert text will also
   * be sent to the console.
   *
   * @param active Set the alert as active and report it to the driver station.
   */
  public void set(boolean active) {
    if (active && !this.active) {
      activeStartTime = Timer.getFPGATimestamp();
      printAlert(text);
    }
    this.active = active;
  }

  /**
   * Updates current alert text.
   *
   * @param text The text for the alert.
   */
  public void setText(String text) {
    if (active && !text.equals(this.text)) {
      printAlert(text);
    }
    this.text = text;
  }

  /**
   * Updates current alert type ( level ).
   *
   * @param level The type for the alert.
   */
  public void setLevel(AlertType level) {
    this.type = level;
  }

  /**
   * Print the alert message.
   *
   * @param text Text to print.
   */
  private void printAlert(String text) {
    switch (type) {
      case ERROR:
        DriverStation.reportError(text, false);
        break;
      case ERROR_TRACE:
        DriverStation.reportError(text, true);
        break;
      case WARNING:
        DriverStation.reportWarning(text, false);
        break;
      case WARNING_TRACE:
        DriverStation.reportWarning(text, true);
        break;
      case INFO:
        System.out.println(text);
        break;
    }
  }

  /** Represents an alert's level of urgency. */
  public static enum AlertType {
    /**
     * High priority alert - displayed first on the dashboard with a red "X" symbol. Use this type
     * for problems which will seriously affect the robot's functionality and thus require immediate
     * attention.
     */
    ERROR,
    /**
     * High priority alert - displayed first on the dashboard with a red "X" symbol. Use this type
     * for problems which will seriously affect the robot's functionality and thus require immediate
     * attention. Trace printed to driver station console.
     */
    ERROR_TRACE,

    /**
     * Medium priority alert - displayed second on the dashboard with a yellow "!" symbol. Use this
     * type for problems which could affect the robot's functionality but do not necessarily require
     * immediate attention.
     */
    WARNING,
    /**
     * Medium priority alert - displayed second on the dashboard with a yellow "!" symbol. Use this
     * type for problems which could affect the robot's functionality but do not necessarily require
     * immediate attention. Trace printed to driver station console.
     */
    WARNING_TRACE,
    /**
     * Low priority alert - displayed last on the dashboard with a green "i" symbol. Use this type
     * for problems which are unlikely to affect the robot's functionality, or any other alerts
     * which do not fall under "ERROR" or "WARNING".
     */
    INFO
  }

  /** Sendable alert for advantage scope. */
  private static class SendableAlerts implements Sendable {

    /** Alert list for sendable. */
    public final List<Alert> alerts = new ArrayList<>();

    /**
     * Get alerts based off of type.
     *
     * @param type Type of alert to fetch.
     * @return Active alert strings.
     */
    public String[] getStrings(AlertType type) {
      List<String> alertStrings = new ArrayList<>();
      for (Alert alert : alerts) {
        alertStrings = getStrings(type, alert, alertStrings);
      }
      // alertStrings.sort((a1, a2) -> (int) (a2.activeStartTime -
      // a1.activeStartTime));
      return alertStrings.toArray(new String[alertStrings.size()]);
    }

    /**
     * Get Strings
     *
     * @param type type
     * @param alert alert
     * @param alertStrings alertStrings
     * @return alertStrings
     */
    private List<String> getStrings(AlertType type, Alert alert, List<String> alertStrings) {
      if (alert.type == type && alert.active) {
        alertStrings.add(alert.text);
      }
      return alertStrings;
    }

    @Override
    public void initSendable(SendableBuilder builder) {
      builder.setSmartDashboardType("Alerts");
      builder.addStringArrayProperty("errors", () -> getStrings(AlertType.ERROR), null);
      builder.addStringArrayProperty("errors", () -> getStrings(AlertType.ERROR_TRACE), null);
      builder.addStringArrayProperty("warnings", () -> getStrings(AlertType.WARNING), null);
      builder.addStringArrayProperty("warnings", () -> getStrings(AlertType.WARNING_TRACE), null);
      builder.addStringArrayProperty("infos", () -> getStrings(AlertType.INFO), null);
    }
  }

  /**
   * Get Activation Timestamp
   *
   * @return active Start Time
   */
  public double getActivationTimestamp() {
    return activeStartTime;
  }

  /**
   * Logs the Alert
   *
   * @param name name
   */
  public void logAlert(String name) {
    Logger.recordMetadata(name, text);
  }
}
