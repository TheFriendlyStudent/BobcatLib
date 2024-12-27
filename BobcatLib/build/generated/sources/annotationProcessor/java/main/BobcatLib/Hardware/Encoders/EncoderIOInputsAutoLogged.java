package BobcatLib.Hardware.Encoders;

import java.lang.Cloneable;
import java.lang.Override;
import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;

public class EncoderIOInputsAutoLogged extends EncoderIO.EncoderIOInputs implements LoggableInputs, Cloneable {
  @Override
  public void toLog(LogTable table) {
    table.put("GetEncoderPosition", getEncoderPosition);
    table.put("Faulted", faulted);
  }

  @Override
  public void fromLog(LogTable table) {
    getEncoderPosition = table.get("GetEncoderPosition", getEncoderPosition);
    faulted = table.get("Faulted", faulted);
  }

  public EncoderIOInputsAutoLogged clone() {
    EncoderIOInputsAutoLogged copy = new EncoderIOInputsAutoLogged();
    copy.getEncoderPosition = this.getEncoderPosition;
    copy.faulted = this.faulted;
    return copy;
  }
}
