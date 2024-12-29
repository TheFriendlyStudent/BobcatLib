package BobcatLib.Hardware.Motors.Utility.CTRE;

import com.ctre.phoenix6.configs.SoftwareLimitSwitchConfigs;

/**
 * A wrapper class for the SoftwareLimitSwitchConfigs that provides immutable updates for reverse
 * limit configuration. Each method in this class returns a new instance with the updated
 * configuration.
 */
public class SoftwareLimitSwitchWrapper {

  private final SoftwareLimitSwitchConfigs softLimitThresh;

  /**
   * Constructs a SoftwareLimitSwitchWrapper with the given SoftwareLimitSwitchConfigs.
   *
   * @param softLimitThresh The initial software limit switch configuration.
   */
  public SoftwareLimitSwitchWrapper(SoftwareLimitSwitchConfigs softLimitThresh) {
    this.softLimitThresh = softLimitThresh;
  }

  /**
   * Returns a new SoftwareLimitSwitchWrapper with the updated reverse soft limit enable and
   * threshold.
   *
   * @param enabled The new reverse soft limit enable value.
   * @param limit The new reverse soft limit threshold value.
   * @return A new SoftwareLimitSwitchWrapper with the updated configuration.
   */
  public SoftwareLimitSwitchWrapper withReverseLimit(boolean enabled, double limit) {
    SoftwareLimitSwitchConfigs newConfig =
        softLimitThresh.withReverseSoftLimitEnable(enabled).withReverseSoftLimitThreshold(limit);
    return new SoftwareLimitSwitchWrapper(newConfig);
  }

  /**
   * Returns a new SoftwareLimitSwitchWrapper with the updated reverse soft limit enable value.
   *
   * @param enabled The new reverse soft limit enable value.
   * @return A new SoftwareLimitSwitchWrapper with the updated reverse soft limit enable.
   */
  public SoftwareLimitSwitchWrapper withReverseLimit(boolean enabled) {
    SoftwareLimitSwitchConfigs newConfig = softLimitThresh.withReverseSoftLimitEnable(enabled);
    return new SoftwareLimitSwitchWrapper(newConfig);
  }

  /**
   * Returns a new SoftwareLimitSwitchWrapper with the updated reverse soft limit threshold value.
   *
   * @param limit The new reverse soft limit threshold value.
   * @return A new SoftwareLimitSwitchWrapper with the updated reverse soft limit threshold.
   */
  public SoftwareLimitSwitchWrapper withReverseLimit(double limit) {
    SoftwareLimitSwitchConfigs newConfig = softLimitThresh.withReverseSoftLimitThreshold(limit);
    return new SoftwareLimitSwitchWrapper(newConfig);
  }

  /**
   * Returns the current SoftwareLimitSwitchConfigs.
   *
   * @return The current SoftwareLimitSwitchConfigs.
   */
  public SoftwareLimitSwitchConfigs getSoftLimitThresh() {
    return softLimitThresh;
  }
}
