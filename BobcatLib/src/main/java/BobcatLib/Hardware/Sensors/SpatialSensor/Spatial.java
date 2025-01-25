package BobcatLib.Hardware.Sensors.SpatialSensor;

import edu.wpi.first.math.geometry.Rotation2d;
import org.littletonrobotics.junction.Logger;

public class Spatial {
  private final SpatialIO io;
  private final SpatialIOInputsAutoLogged inputs = new SpatialIOInputsAutoLogged();
  public boolean isEnabled = false;

  /** continuously gets the distances */
  public Spatial(SpatialIO io) {
    this.io = io;
  }

  public void periodic(Rotation2d angle) {
    io.updateInputs(inputs, angle, isEnabled);
    Logger.processInputs("Spatial", inputs);
  }
}
