package BobcatLib.Hardware.Sensors.TOFs;

import java.lang.Cloneable;
import java.lang.Override;
import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;

public class TimeOfFlightIOInputsAutoLogged extends TimeOfFlightIO.TimeOfFlightIOInputs implements LoggableInputs, Cloneable {
  @Override
  public void toLog(LogTable table) {
    table.put("Distance", distance);
  }

  @Override
  public void fromLog(LogTable table) {
    distance = table.get("Distance", distance);
  }

  public TimeOfFlightIOInputsAutoLogged clone() {
    TimeOfFlightIOInputsAutoLogged copy = new TimeOfFlightIOInputsAutoLogged();
    copy.distance = this.distance;
    return copy;
  }
}
