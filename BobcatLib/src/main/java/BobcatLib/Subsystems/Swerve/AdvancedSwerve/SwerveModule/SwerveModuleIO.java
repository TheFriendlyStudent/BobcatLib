package BobcatLib.Subsystems.Swerve.AdvancedSwerve.SwerveModule;

import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.geometry.Rotation2d;
import org.littletonrobotics.junction.AutoLog;

public interface SwerveModuleIO {
  @AutoLog
  public static class SwerveModuleIOInputs {
    public Rotation2d offset = new Rotation2d();

    public double drivePositionRot = -1.0;
    public double driveVelocityRotPerSec = -1.0;
    public double driveAccelerationRadPerSecSquared = -1.0;

    public double canCoderPositionRot = -1.0;
    public double canCoderPositionDeg = -1.0;
    public double turnAngularVelocityRadPerSec = -1.0;

    public double internalTempDrive = -1.0;
    public double processorTempDrive = -1.0;
    public double internalTempAngle = -1.0;
    public double processorTempAngle = -1.0;
    public double appliedDriveVoltage = -1.0;
    public double driveCurrentAmps = -1;
    public double angleCurrentAmps = -1.0;
    public double angleAppliedVoltage = -1.0;

    public double[] odometryTimestamps = new double[] {};
    public double[] odometryDrivePositionsRad = new double[] {};
    public Rotation2d[] odometryAnglePositions = new Rotation2d[] {};
  }

  public default void updateInputs(SwerveModuleIOInputs inputs) {}

  /**
   * Sets the velocity of the drive motor
   *
   * @param velocityRadPerSec velocity to set it to
   */
  public default void setDriveVelocity(Rotation2d velocityRadPerSec) {}

  /** Stops the drive motor */
  public default void stopDrive() {}

  /**
   * Sets the neutral mode of the drive motor
   *
   * @param mode mode to set it to
   */
  public default void setDriveNeutralMode(NeutralModeValue mode) {}

  /**
   * Sets the angle position of the angle motor
   *
   * @param rotation angle to set it to
   */
  public default void setAnglePosition(Rotation2d rotation) {}

  /** Stops the angle motor */
  public default void stopAngle() {}

  /**
   * Sets the neutral mode of the angle motor
   *
   * @param mode mode to set it to
   */
  public default void setAngleNeutralMode(NeutralModeValue mode) {}

  public default void runCharachterization(double volts) {}
}
