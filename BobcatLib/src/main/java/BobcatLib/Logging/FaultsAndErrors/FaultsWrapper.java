package BobcatLib.Logging.FaultsAndErrors;

import BobcatLib.Logging.Alert;
import BobcatLib.Logging.Alert.AlertType;

/** Faults Wrapper */
public interface FaultsWrapper {
  /**
   * @param alert
   */
  public default void activateAlert(Alert alert) {}

  /**
   * @param alert
   * @param type
   */
  public default void activateAlert(Alert alert, AlertType type) {}

  /**
   * @param alert
   */
  public default void disableAlert(Alert alert) {}

  /**
   * @return
   */
  public default boolean hasFaultOccured() {
    return false;
  }
}
