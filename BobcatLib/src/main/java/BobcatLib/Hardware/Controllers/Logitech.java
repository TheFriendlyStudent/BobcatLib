package BobcatLib.Hardware.Controllers;

import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class Logitech implements ControllerWrapper {
  CommandJoystick logitechJoystick;
  /** Controller Axis Mappings */
  public final int translationAxis = 0;

  public final int strafeAxis = 1;
  public final int rotationAxis = 2;

  /**
   * Contains Left Joystick , Right Joystick , POV , triangle, circle, cross, square buttons.
   *
   * @param port
   */
  public Logitech(int port) {
    logitechJoystick = new CommandJoystick(port);
  }

  /**
   * Gets the axis value for translation (forward/backward) control.
   *
   * @return The axis value for translation control.
   */
  public double getTranslationAxis() {
    return logitechJoystick.getRawAxis(translationAxis);
  }

  /**
   * Gets the axis value for strafing (side-to-side) control.
   *
   * @return The axis value for strafing control.
   */
  public double getStrafeAxis() {
    return logitechJoystick.getRawAxis(strafeAxis);
  }

  /**
   * Gets the axis value for rotation control.
   *
   * @return The axis value for rotation control.
   */
  public double getRotationAxis() {
    return logitechJoystick.getRawAxis(rotationAxis);
  }

  public Trigger getLeftTrigger() {
    return new Trigger(() -> false);
  }

  public Trigger getRightTrigger() {
    return new Trigger(() -> false);
  }

  public Trigger getLeftBumper() {
    return logitechJoystick.button(5);
  }

  public Trigger getRightBumper() {
    return logitechJoystick.button(6);
  }

  public Trigger getYorTriangle() {
    return logitechJoystick.button(4);
  }

  public Trigger getBorCircle() {
    return logitechJoystick.button(3);
  }

  public Trigger getAorCross() {
    return logitechJoystick.button(2);
  }

  public Trigger getXorSquare() {
    return logitechJoystick.button(1);
  }

  public Trigger getDPadTriggerUp() {
    return logitechJoystick.povUp();
  }

  public Trigger getDPadTriggerDown() {
    return logitechJoystick.povDown();
  }

  public Trigger getDPadTriggerLeft() {
    return logitechJoystick.povLeft();
  }

  public Trigger getDPadTriggerRight() {
    return logitechJoystick.povRight();
  }
}
