package BobcatLib.Logging.FaultsAndErrors;

import BobcatLib.Logging.Alert;
import BobcatLib.Logging.Alert.AlertType;

/**
 * Interface for managing hardware fault alerts. Provides methods to activate, deactivate, and check
 * for faults in the system.
 */
public interface FaultsWrapper {

  /**
   * Activates a specific alert, indicating that a fault condition has been detected.
   *
   * @param alert The alert to activate.
   */
  public default void activateAlert(Alert alert) {}

  /**
   * Activates a specific alert with a defined severity level.
   *
   * @param alert The alert to activate.
   * @param type The severity level of the alert.
   */
  public default void activateAlert(Alert alert, AlertType type) {}

  /**
   * Deactivates a specific alert, indicating that the fault condition no longer exists.
   *
   * @param alert The alert to deactivate.
   */
  public default void disableAlert(Alert alert) {}

  /**
   * Checks if any fault conditions have been detected.
   *
   * @return {@code true} if at least one fault is detected; otherwise, {@code false}.
   */
  public default boolean hasFaultOccured() {
    return false;
  }
}
