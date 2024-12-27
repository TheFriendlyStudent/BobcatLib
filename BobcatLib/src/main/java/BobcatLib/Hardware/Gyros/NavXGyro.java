package BobcatLib.Hardware.Gyros;

import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.SerialPort.Port;

/** Represents an interface for interacting with a gyro sensor. */
public class NavXGyro implements GyroIO {
  private final AHRS gyro;

  public NavXGyro() {
    gyro = new AHRS(Port.kUSB);
    setYaw(0);
  }

  /**
   * Updates the gyro inputs based on external sources.
   *
   * @param inputs The inputs to update.
   */
  public void updateInputs(GyroIOInputs inputs) {
    inputs.yawPosition = Rotation2d.fromRadians(getYaw());
    inputs.rollPosition = Rotation2d.fromRadians(getRoll());
    inputs.pitchPosition = Rotation2d.fromRadians(getPitch());
  }

  /**
   * Sets the yaw value of the gyro sensor.
   *
   * @param yaw The yaw value to set (in degrees).
   */
  public void setYaw(double yaw) {
    gyro.zeroYaw();
  }
  /**
   * Sets the pitch value of the gyro sensor.
   *
   * @param pitch The pitch value to set (in degrees).
   */
  public void setPitch(double pitch) {
    gyro.zeroYaw();
  }
  /**
   * Sets the roll value of the gyro sensor.
   *
   * @param roll The roll value to set (in degrees).
   */
  public void setRoll(double roll) {
    gyro.zeroYaw();
  }

  private double getYaw() {
    return gyro.getRotation3d().getZ();
  }

  private double getPitch() {
    return gyro.getRotation3d().getY();
  }

  private double getRoll() {
    return gyro.getRotation3d().getX();
  }

  public void handleFaults() {}
}
