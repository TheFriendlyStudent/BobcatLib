package BobcatLib.Hardware.Controllers;

import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class Ruffy implements ControllerWrapper {
  CommandJoystick ruffyJoystick;
  /** Controller Axis Mappings */
  public final int translationAxis = 0;

  public final int strafeAxis = 1;
  public final int rotationAxis = 2;

  /**
   * Contains Left Joystick , Right Joystick , POV , triangle, circle, cross, square buttons.
   *
   * @param port
   */
  public Ruffy(int port) {
    ruffyJoystick = new CommandJoystick(port);
  }

  /**
   * Gets the axis value for translation (forward/backward) control.
   *
   * @return The axis value for translation control.
   */
  public double getTranslationAxis() {
    return ruffyJoystick.getRawAxis(translationAxis);
  }

  /**
   * Gets the axis value for strafing (side-to-side) control.
   *
   * @return The axis value for strafing control.
   */
  public double getStrafeAxis() {
    return ruffyJoystick.getRawAxis(strafeAxis);
  }

  /**
   * Gets the axis value for rotation control.
   *
   * @return The axis value for rotation control.
   */
  public double getRotationAxis() {
    return ruffyJoystick.getRawAxis(rotationAxis);
  }

  public Trigger getLeftTrigger() {
    return new Trigger(() -> false);
  }

  public Trigger getRightTrigger() {
    return new Trigger(() -> false);
  }

  public Trigger getLeftBumper() {
    return new Trigger(() -> false);
  }

  public Trigger getRightBumper() {
    return new Trigger(() -> false);
  }

  public Trigger getYorTriangle() {
    return new Trigger(() -> false);
  }

  public Trigger getBorCircle() {
    return new Trigger(() -> false);
  }

  public Trigger getAorCross() {
    return new Trigger(() -> false);
  }

  public Trigger getXorSquare() {
    return new Trigger(() -> false);
  }

  public Trigger getDPadTriggerUp() {
    return new Trigger(() -> false);
  }

  public Trigger getDPadTriggerDown() {
    return new Trigger(() -> false);
  }

  public Trigger getDPadTriggerLeft() {
    return new Trigger(() -> false);
  }

  public Trigger getDPadTriggerRight() {
    return new Trigger(() -> false);
  }
}
