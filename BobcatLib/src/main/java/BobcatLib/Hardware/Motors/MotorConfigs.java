package BobcatLib.Hardware.Motors;

import BobcatLib.Hardware.Motors.SensorHelpers.NeutralModeWrapper;
import BobcatLib.Hardware.Motors.SensorHelpers.SensorDirectionWrapper;
import BobcatLib.Utilities.CANDeviceDetails;

public class MotorConfigs {
  public double motorToGearRatio;
  public NeutralModeWrapper.disabledMode mode;
  public SensorDirectionWrapper sensorDirection;
  public boolean isInverted;
  public double kP;
  public double kI;
  public double kD;
  public OptionalConfigCtre optionalCtre = new OptionalConfigCtre();
  public OptionalConfigRev optionalRev = new OptionalConfigRev();
  public CANDeviceDetails canDevice;
}
