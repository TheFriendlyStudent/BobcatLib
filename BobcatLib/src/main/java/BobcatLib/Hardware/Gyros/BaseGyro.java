package BobcatLib.Hardware.Gyros;

import edu.wpi.first.math.geometry.Rotation2d;
import org.littletonrobotics.junction.Logger;

public class BaseGyro {
  private final GyroIO io;
  private final GyroIOInputsAutoLogged inputs = new GyroIOInputsAutoLogged();
  private final String name;

  public BaseGyro(String name, GyroIO io) {
    this.io = io;
    this.name = name;
  }

  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs(name, inputs);
  }

  public void setYaw(double yaw) {
    io.setYaw(yaw);
  }

  public void setRoll(double roll) {
    io.setRoll(roll);
  }

  public void setPitch(double pitch) {
    io.setPitch(pitch);
  }

  public Rotation2d getYaw() {
    return inputs.yawPosition;
  }

  public Rotation2d getPitch() {
    return inputs.pitchPosition;
  }

  public Rotation2d getRoll() {
    return inputs.rollPosition;
  }

  public double getTimeDiff() {
    return io.getTimeDiff();
  }
}
