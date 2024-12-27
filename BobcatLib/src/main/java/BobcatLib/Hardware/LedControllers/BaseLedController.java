package BobcatLib.Hardware.LedControllers;

import BobcatLib.Hardware.LedControllers.LedControllerIO.LedControllerIOInputs;

public class BaseLedController {
  private final LedControllerIO io;
  private final LedControllerIOInputs inputs = new LedControllerIOInputs();
  private final String name;

  public BaseLedController(String name, LedControllerIO io) {
    this.io = io;
    this.name = name;
  }

  public void periodic() {
    io.updateInputs(inputs);
  }

  public void checkForFaults() {
    io.checkForFaults();
  }
}
