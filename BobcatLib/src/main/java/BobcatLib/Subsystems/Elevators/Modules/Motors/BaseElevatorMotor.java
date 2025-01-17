package BobcatLib.Subsystems.Elevators.Modules.Motors;

public interface BaseElevatorMotor {
    
  public default void configAngleMotor() {
  }

  /*
   * sets the angle of the motor by setting the internal pid.
   *
   * @param rotations
   */
  public default void setAngle(double rotations) {
  }

  public default void setControl(double percent) {
  }

  /**
   * gets the position of the steer motor.
   *
   * @return position of the steer motor in rotations.
   */
  public default  double getPosition() {
    return 0.00;
}


  public default  double getErrorPosition() {
    return 0.00;
  }

  /**
   * sets the position of the steer motor given the desired rotations.
   *
   * @param absolutePosition
   */
  public default void setPosition(double absolutePosition) {
  }

  /** Stops the motors properly. */
  public default void stopMotor() {
  }
}
