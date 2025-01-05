package BobcatLib.Subsystems.Swerve.AdvancedSwerve.SwerveModule;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import org.littletonrobotics.junction.Logger;

public class SwerveModuleIOSim implements SwerveModuleIO {
  private DCMotorSim driveSim;
  private DCMotorSim angleSim;
  private static final DCMotor DRIVE_GEARBOX = DCMotor.getKrakenX60Foc(1);
  private static final DCMotor TURN_GEARBOX = DCMotor.getKrakenX60Foc(1);

  private double angleAbsolutePosRot = Math.random();
  private double loopPeriodSecs;
  private double driveInertia = 0.0001;
  private double steerInertia = 0.025;

  public SwerveModuleIOSim(double loopPeriodSecs, double driveGearRatio, double angleGearRatio) {

    // Using flywheels to simulate motors TODO update this
    driveSim =
        new DCMotorSim(
            LinearSystemId.createDCMotorSystem(DRIVE_GEARBOX, driveInertia, driveGearRatio),
            DRIVE_GEARBOX);
    angleSim =
        new DCMotorSim(
            LinearSystemId.createDCMotorSystem(TURN_GEARBOX, steerInertia, driveGearRatio),
            TURN_GEARBOX);

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
    Logger.recordOutput("Swerve/Debug/DriveVolts", volts);
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
    Logger.recordOutput("iwannakms", true);

    double volts = MathUtil.clamp(percent * 12, -12.0, 12.0);
    Logger.recordOutput("Swerve/Debug/AngleVolts", volts);
    angleSim.setInputVoltage(volts);
  }

  /** Stops the angle motor */
  public void stopAngle() {
    angleSim.setInputVoltage(0);
  }
}
