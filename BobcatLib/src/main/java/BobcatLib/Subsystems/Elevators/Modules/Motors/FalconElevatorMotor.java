package BobcatLib.Subsystems.Elevators.Modules.Motors;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

public class FalconElevatorMotor implements BaseElevatorMotor {
  private TalonFX mAngleMotor;
  private TalonFXConfiguration swerveAngleFXConfig = new TalonFXConfiguration();
  /* angle motor control requests */
  private final PositionVoltage anglePosition = new PositionVoltage(0);

  public FalconElevatorMotor(int id, String canivorename) {
    if (canivorename == "") {

      mAngleMotor = new TalonFX(id);
    } else {

      mAngleMotor = new TalonFX(id, canivorename);
    }

    configAngleMotor();
    mAngleMotor.getConfigurator().apply(swerveAngleFXConfig);
  }

  public void configAngleMotor() {
    /** Swerve Angle Motor Configurations */
    /* Motor Inverts and Neutral Mode */
    swerveAngleFXConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    swerveAngleFXConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;

    /* Gear Ratio and Wrapping Config */

    swerveAngleFXConfig.Feedback.SensorToMechanismRatio = 12;
    swerveAngleFXConfig.ClosedLoopGeneral.ContinuousWrap = false;
    /* Current Limiting */
    swerveAngleFXConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
    swerveAngleFXConfig.CurrentLimits.SupplyCurrentLimit = 40;

    /* PID Config */
    swerveAngleFXConfig.Slot0.kP = 0.125;
    swerveAngleFXConfig.Slot0.kI = 0;
    swerveAngleFXConfig.Slot0.kD = 0;

    /* Open and Closed Loop Ramping */
    swerveAngleFXConfig.ClosedLoopRamps.withVoltageClosedLoopRampPeriod(0.0);
    swerveAngleFXConfig.OpenLoopRamps.withVoltageOpenLoopRampPeriod(0.25);
  }

  /*
   * sets the angle of the motor by setting the internal pid.
   *
   * @param rotations
   */
  public void setAngle(double rotations) {
    mAngleMotor.setControl(anglePosition.withPosition(rotations));
  }

  public void setControl(double percent) {
    mAngleMotor.set(percent);
  }

  /**
   * gets the position of the steer motor.
   *
   * @return position of the steer motor in rotations.
   */
  public double getPosition() {
    return mAngleMotor.getPosition().getValueAsDouble();
  }

  public double getErrorPosition() {
    return mAngleMotor.getClosedLoopError().getValueAsDouble();
  }

  /**
   * sets the position of the steer motor given the desired rotations.
   *
   * @param absolutePosition
   */
  public void setPosition(double absolutePosition) {
    mAngleMotor.setPosition(absolutePosition);
  }

  /** Stops the motors properly. */
  public void stopMotor() {
    mAngleMotor.stopMotor();
  }
}
