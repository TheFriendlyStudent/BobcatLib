package BobcatLib.Hardware.Sensors.TOFs;

import org.littletonrobotics.junction.Logger;

public class BaseTof {
  private final TimeOfFlightIO io;
  private final TimeOfFlightIOInputsAutoLogged inputs = new TimeOfFlightIOInputsAutoLogged();
  private final String name;

  public BaseTof(String name, TimeOfFlightIO io) {
    this.io = io;
    this.name = name;
  }

  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs(name, inputs);
  }

  public double getRange() {
    return io.getRange();
  }
}
