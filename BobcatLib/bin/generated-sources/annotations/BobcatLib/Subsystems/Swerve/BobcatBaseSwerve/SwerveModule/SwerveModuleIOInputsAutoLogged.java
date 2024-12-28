package BobcatLib.Subsystems.Swerve.BobcatBaseSwerve.SwerveModule;

import java.lang.Cloneable;
import java.lang.Override;
import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;

public class SwerveModuleIOInputsAutoLogged extends SwerveModuleIO.SwerveModuleIOInputs implements LoggableInputs, Cloneable {
  @Override
  public void toLog(LogTable table) {
    table.put("Offset", offset);
    table.put("DrivePositionRot", drivePositionRot);
    table.put("DriveVelocityRotPerSec", driveVelocityRotPerSec);
    table.put("DriveAcceleration", driveAcceleration);
    table.put("CanCoderPositionRot", canCoderPositionRot);
    table.put("RawCanCoderPositionDeg", rawCanCoderPositionDeg);
    table.put("InternalTempDrive", internalTempDrive);
    table.put("ProcessorTempDrive", processorTempDrive);
    table.put("InternalTempAngle", internalTempAngle);
    table.put("ProcessorTempAngle", processorTempAngle);
    table.put("AppliedDriveVoltage", appliedDriveVoltage);
    table.put("OdometryTimestamps", odometryTimestamps);
    table.put("OdometryDrivePositionsRad", odometryDrivePositionsRad);
    table.put("OdometryAnglePositions", odometryAnglePositions);
  }

  @Override
  public void fromLog(LogTable table) {
    offset = table.get("Offset", offset);
    drivePositionRot = table.get("DrivePositionRot", drivePositionRot);
    driveVelocityRotPerSec = table.get("DriveVelocityRotPerSec", driveVelocityRotPerSec);
    driveAcceleration = table.get("DriveAcceleration", driveAcceleration);
    canCoderPositionRot = table.get("CanCoderPositionRot", canCoderPositionRot);
    rawCanCoderPositionDeg = table.get("RawCanCoderPositionDeg", rawCanCoderPositionDeg);
    internalTempDrive = table.get("InternalTempDrive", internalTempDrive);
    processorTempDrive = table.get("ProcessorTempDrive", processorTempDrive);
    internalTempAngle = table.get("InternalTempAngle", internalTempAngle);
    processorTempAngle = table.get("ProcessorTempAngle", processorTempAngle);
    appliedDriveVoltage = table.get("AppliedDriveVoltage", appliedDriveVoltage);
    odometryTimestamps = table.get("OdometryTimestamps", odometryTimestamps);
    odometryDrivePositionsRad = table.get("OdometryDrivePositionsRad", odometryDrivePositionsRad);
    odometryAnglePositions = table.get("OdometryAnglePositions", odometryAnglePositions);
  }

  public SwerveModuleIOInputsAutoLogged clone() {
    SwerveModuleIOInputsAutoLogged copy = new SwerveModuleIOInputsAutoLogged();
    copy.offset = this.offset;
    copy.drivePositionRot = this.drivePositionRot;
    copy.driveVelocityRotPerSec = this.driveVelocityRotPerSec;
    copy.driveAcceleration = this.driveAcceleration;
    copy.canCoderPositionRot = this.canCoderPositionRot;
    copy.rawCanCoderPositionDeg = this.rawCanCoderPositionDeg;
    copy.internalTempDrive = this.internalTempDrive;
    copy.processorTempDrive = this.processorTempDrive;
    copy.internalTempAngle = this.internalTempAngle;
    copy.processorTempAngle = this.processorTempAngle;
    copy.appliedDriveVoltage = this.appliedDriveVoltage;
    copy.odometryTimestamps = this.odometryTimestamps.clone();
    copy.odometryDrivePositionsRad = this.odometryDrivePositionsRad.clone();
    copy.odometryAnglePositions = this.odometryAnglePositions.clone();
    return copy;
  }
}
