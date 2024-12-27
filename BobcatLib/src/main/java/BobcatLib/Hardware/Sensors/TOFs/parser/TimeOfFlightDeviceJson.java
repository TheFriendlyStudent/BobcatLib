package BobcatLib.Hardware.Sensors.TOFs.parser;

import BobcatLib.Hardware.Sensors.TOFs.Utility.RangingMode;

public class TimeOfFlightDeviceJson {
  public RangingMode mode;
  public double sampleTime;
  public int tofId;

  public TimeOfFlightDeviceJson(RangingMode mode, int tofId, double sampleTime) {
    this.mode = mode;
    this.sampleTime = sampleTime;
    this.tofId = tofId;
  }
}
