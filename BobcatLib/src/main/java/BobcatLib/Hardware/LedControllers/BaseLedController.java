package BobcatLib.Hardware.LedControllers;

import org.littletonrobotics.junction.Logger;

public class BaseLedController {
  private final LedControllerIO io;
  private final LedControllerIOInputsAutoLogged inputs = new LedControllerIOInputsAutoLogged();
  private final String name;

  public BaseLedController(String name, LedControllerIO io) {
    this.io = io;
    this.name = name;
  }

  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs(name, inputs);
  }

  public void checkForFaults() {
    io.checkForFaults();
  }
}
