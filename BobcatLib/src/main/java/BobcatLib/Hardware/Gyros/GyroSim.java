package BobcatLib.Hardware.Gyros;

import com.ctre.phoenix6.hardware.Pigeon2;
import com.studica.frc.*;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Timer;

public class GyroSim implements GyroIO {
  /** Main timer to control movement estimations. */
  private Timer timer;
  /** The last time the timer was read, used to determine position changes. */
  private double lastTime;
  /** Heading of the robot. */
  private double yawAngle, pitchAngle, rollAngle;

  public GyroSim() {
    timer = new Timer();
    timer.start();
    lastTime = timer.get();
    configGyro();
  }

  public void configGyro() {}

  /**
   * Updates the gyro inputs based on external sources.
   *
   * @param inputs The inputs to update.
   */
  public void updateInputs(GyroIOInputs inputs) {
    inputs.yawPosition = Rotation2d.fromDegrees(getYaw());
    inputs.rollPosition = Rotation2d.fromDegrees(getRoll());
    inputs.pitchPosition = Rotation2d.fromDegrees(getPitch());
  }

  /**
   * Sets the yaw value of the gyro sensor.
   *
   * @param yaw The yaw value to set (in degrees).
   */
  public void setYaw(double yaw) {
    this.yawAngle = yaw;
  }

  /**
   * Sets the Pitch value of the gyro sensor.
   *
   * @param pitch The pitch value to set (in degrees).
   */
  public void setPitch(double pitch) {
    this.pitchAngle = pitch;
  }

  /**
   * Sets the roll value of the gyro sensor.
   *
   * @param roll The roll value to set (in degrees).
   */
  public void setRoll(double roll) {
    this.rollAngle = roll;
  }

  public void periodic(Pigeon2 imu) {}

  public void periodic(com.studica.frc.AHRS imu) {}

  public void periodic() {}

  private double getYaw() {
    return yawAngle;
  }

  private double getPitch() {
    return pitchAngle;
  }

  private double getRoll() {
    return rollAngle;
  }

  public double getTimeDiff() {
    return (timer.get() - lastTime);
  }
}
