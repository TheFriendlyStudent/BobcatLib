package BobcatLib.Subsystems.Elevators.Modules;

import BobcatLib.Hardware.Motors.BaseMotor;
import BobcatLib.Hardware.Motors.MotorIO;
import BobcatLib.Hardware.Motors.Utility.SoftwareLimitWrapper;
import BobcatLib.Subsystems.Elevators.Utility.Parser.ElevatorJson;
import edu.wpi.first.math.geometry.Rotation2d;

/**
 * The {@code ElevatorReal} class provides a concrete implementation of an elevator mechanism using
 * a motor and software limit switches. This class handles motor control and ensures the elevator
 * operates within predefined limits.
 */
public class ElevatorReal implements ElevatorIO {

  private BaseMotor elevatorMotor;
  private SoftwareLimitWrapper limitWrapper;

  /**
   * Constructs a new {@code ElevatorReal} instance.
   *
   * @param motor The {@link MotorIO} instance used for motor control.
   * @param limits The {@link SoftwareLimitWrapper} instance defining the elevator's limits.
   */
  public ElevatorReal(MotorIO motor, SoftwareLimitWrapper limits) {
    this.limitWrapper = limits;
    elevatorMotor = new BaseMotor(motor, this.limitWrapper);
  }

  /**
   * Configures the elevator with default settings. This method can be used to set initial motor or
   * limit configurations.
   */
  public void configClimber() {}

  /**
   * Loads the elevator configuration from a JSON file.
   *
   * @param json The {@link ElevatorJson} instance containing the elevator configuration.
   */
  public void loadConfigurationFromFile(ElevatorJson json) {
    limitWrapper =
        new SoftwareLimitWrapper(
            json.limits.lowerLimits,
            json.limits.upperLimits,
            SoftwareLimitWrapper.SoftwareLimitType.BOTH);
  }

  /**
   * Sets the motor's output power as a percentage of its maximum power.
   *
   * @param percent The desired output power (-1.0 to 1.0).
   */
  public void setPercentOut(double percent) {
    elevatorMotor.setControl(percent);
  }

  /** Holds the elevator in its current position using position control. */
  public void holdPos() {
    elevatorMotor.setAngle(getPosition().getRotations());
  }

  /**
   * Sets the elevator's position to a specified rotation value. If the requested position exceeds
   * the upper limit, the elevator will hold its current position.
   *
   * @param rot The desired position in rotations.
   */
  public void setPosition(double rot) {
    if (getPosition().getRotations() < limitWrapper.getUpperLimit().getRotations()) {
      elevatorMotor.setAngle(rot);
      return;
    }
    holdPos();
  }

  /** Stops the elevator motor by setting its output to zero. */
  public void stop() {
    elevatorMotor.stopMotor();
  }

  /**
   * Retrieves the current position of the elevator.
   *
   * @return The current position as a {@link Rotation2d} instance.
   */
  public Rotation2d getPosition() {
    return elevatorMotor.getPosition();
  }
}
