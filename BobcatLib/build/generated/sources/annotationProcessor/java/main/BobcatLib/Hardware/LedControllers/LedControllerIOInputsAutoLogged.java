package BobcatLib.Hardware.LedControllers;

import java.lang.Cloneable;
import java.lang.Override;
import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;

public class LedControllerIOInputsAutoLogged extends LedControllerIO.LedControllerIOInputs implements LoggableInputs, Cloneable {
  @Override
  public void toLog(LogTable table) {
    table.put("CurrState", currState);
  }

  @Override
  public void fromLog(LogTable table) {
    currState = table.get("CurrState", currState);
  }

  public LedControllerIOInputsAutoLogged clone() {
    LedControllerIOInputsAutoLogged copy = new LedControllerIOInputsAutoLogged();
    copy.currState = this.currState;
    return copy;
  }
}
