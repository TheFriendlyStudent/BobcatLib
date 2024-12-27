package BobcatLib.Hardware.Motors;

public class OptionalConfigCtre {
  public class RampAndLoopConfiguration {
    private final double DutyCycleRampPeriod;
    private final double VoltageRampPeriod;
    private final double TorqueRampPeriod;

    /**
     * Constructs an immutable RampAndLoopConfiguration with the specified ramp periods.
     *
     * @param dutycycle The duty cycle ramp period.
     * @param torque The torque cycle ramp period.
     * @param voltage The voltage ramp period.
     */
    public RampAndLoopConfiguration(double dutycycle, double voltage, double torque) {
      this.DutyCycleRampPeriod = dutycycle;
      this.TorqueRampPeriod = torque;
      this.VoltageRampPeriod = voltage;
    }

    /**
     * Constructs an immutable RampAndLoopConfiguration with the specified ramp periods.
     *
     * @param dutycycle The duty cycle ramp period.
     * @param voltage The voltage ramp period.
     */
    public RampAndLoopConfiguration(double dutycycle, double voltage) {
      this.DutyCycleRampPeriod = dutycycle;
      this.TorqueRampPeriod = 0.00;
      this.VoltageRampPeriod = voltage;
    }

    /**
     * Retrieves the duty cycle ramp period.
     *
     * @return The duty cycle ramp period.
     */
    public double getDutyCycleRampPeriod() {
      return DutyCycleRampPeriod;
    }

    /**
     * Retrieves the voltage ramp period.
     *
     * @return The voltage ramp period.
     */
    public double getVoltageRampPeriod() {
      return VoltageRampPeriod;
    }

    /**
     * Retrieves the torque ramp period.
     *
     * @return The torque ramp period.
     */
    public double getTorqueRampPeriod() {
      return TorqueRampPeriod;
    }
  }

  public boolean useFOC;
  public boolean StatorSupplyCurrentEnable;
  public double StatorSupplyCurrentLimit;
  public boolean SupplyCurrentLimitEnable;
  public double SupplyCurrentLimit;
  public double SupplyCurrentThreshold;
  public double SupplyTimeThreshold;
  public RampAndLoopConfiguration openLoop;
  public RampAndLoopConfiguration closedLoop;
}
