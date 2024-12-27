package BobcatLib.Hardware.Gyros;

import com.ctre.phoenix6.hardware.Pigeon2;
import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.math.geometry.Rotation2d;
import org.littletonrobotics.junction.AutoLog;

/** Represents an interface for interacting with a gyro sensor. */
public interface GyroIO {
  /** Represents the inputs for the gyro sensor. */
  @AutoLog
  public static class GyroIOInputs {
    /** Indicates if the gyro is connected. */
    public boolean connected = false;
    /** The current yaw position of the gyro. */
    public Rotation2d yawPosition = new Rotation2d();
    /** The current pitch position of the gyro. */
    public Rotation2d pitchPosition = new Rotation2d();
    /** The current roll position of the gyro. */
    public Rotation2d rollPosition = new Rotation2d();
    /** Indicates if the gyro has a fault. */
    public boolean faulted = false;
  }

  /**
   * Updates the gyro inputs based on external sources.
   *
   * @param inputs The inputs to update.
   */
  public default void updateInputs(GyroIOInputs inputs) {}

  public default double getTimeDiff() {
    return 1.0;
  }

  /**
   * Sets the yaw value of the gyro sensor.
   *
   * @param yaw The yaw value to set (in degrees).
   */
  public default void setYaw(double yaw) {}
  /**
   * Sets the pitch value of the gyro sensor.
   *
   * @param pitch The pitch value to set (in degrees).
   */
  public default void setPitch(double pitch) {}
  /**
   * Sets the roll value of the gyro sensor.
   *
   * @param roll The roll value to set (in degrees).
   */
  public default void setRoll(double roll) {}

  public default void periodic(Pigeon2 imu) {}

  public default void periodic(AHRS imu) {}

  public default void periodic() {}
}
