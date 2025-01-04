package BobcatLib.Subsystems.Swerve.AdvancedSwerve.SwerveModule;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;

public class SwerveModuleIOSim implements SwerveModuleIO {
  private FlywheelSim driveSim;
  private FlywheelSim angleSim;

  private double angleAbsolutePosRot = Math.random();
  private double loopPeriodSecs;

  public SwerveModuleIOSim(double loopPeriodSecs, double driveGearRatio, double angleGearRatio) {

    double moi = 0.5 * 0.25 * Units.inchesToMeters(4) * Units.inchesToMeters(4);

    // Using flywheels to simulate motors TODO update this
    driveSim =
        new FlywheelSim(
            LinearSystemId.createFlywheelSystem(DCMotor.getFalcon500Foc(1), moi, driveGearRatio),
            DCMotor.getFalcon500Foc(1),
            null);
    angleSim =
        new FlywheelSim(
            LinearSystemId.createFlywheelSystem(DCMotor.getFalcon500Foc(1), moi, driveGearRatio),
            DCMotor.getFalcon500Foc(1),
            null);
    this.loopPeriodSecs = loopPeriodSecs;
  }

  public void updateInputs(SwerveModuleIOInputs inputs) {
    driveSim.update(loopPeriodSecs);
    angleSim.update(loopPeriodSecs);

    double angleDiffRot = (angleSim.getAngularVelocityRPM() / 60) * loopPeriodSecs;
    angleAbsolutePosRot += angleDiffRot;
    while (angleAbsolutePosRot < 0) {
      angleAbsolutePosRot += 1;
    }
    while (angleAbsolutePosRot > 1) {
      angleAbsolutePosRot -= 1;
    }
    inputs.drivePositionRot += (driveSim.getAngularVelocityRPM() / 60) * loopPeriodSecs;
    inputs.driveVelocityRotPerSec = driveSim.getAngularVelocityRPM() / 60;
    inputs.canCoderPositionRot = angleAbsolutePosRot;
  }

  /**
   * Sets the percent out of the drive motor
   *
   * @param percent percent to set it to, from -1.0 to 1.0
   */
  public void setDrivePercentOut(double percent) {
    double volts = MathUtil.clamp(percent * 12, -12.0, 12.0);
    driveSim.setInputVoltage(volts);
  }

  /** Stops the drive motor */
  public void stopDrive() {
    driveSim.setInputVoltage(0);
  }

  /**
   * Sets the percent out of the angle motor
   *
   * @param percent percent to set it to, from -1.0 to 1.0
   */
  public void setAnglePercentOut(double percent) {
    double volts = MathUtil.clamp(percent * 12, -12.0, 12.0);
    angleSim.setInputVoltage(volts);
  }

  /** Stops the angle motor */
  public void stopAngle() {
    angleSim.setInputVoltage(0);
  }
}
