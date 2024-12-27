package BobcatLib.Hardware.Controllers;

import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class EightBitDo implements ControllerWrapper {
  CommandJoystick ebdJoystick;
  /** Controller Axis Mappings */
  public final int translationAxis = 0;

  public final int strafeAxis = 1;
  public final int rotationAxis = 4;

  /**
   * Contains Left Joystick , Right Joystick , POV , triangle, circle, cross, square buttons.
   *
   * @param port
   */
  public EightBitDo(int port) {
    ebdJoystick = new CommandJoystick(port);
  }

  /**
   * Gets the axis value for translation (forward/backward) control.
   *
   * @return The axis value for translation control.
   */
  public double getTranslationAxis() {
    return ebdJoystick.getRawAxis(translationAxis);
  }

  /**
   * Gets the axis value for strafing (side-to-side) control.
   *
   * @return The axis value for strafing control.
   */
  public double getStrafeAxis() {
    return ebdJoystick.getRawAxis(strafeAxis);
  }

  /**
   * Gets the axis value for rotation control.
   *
   * @return The axis value for rotation control.
   */
  public double getRotationAxis() {
    return ebdJoystick.getRawAxis(rotationAxis);
  }

  public Trigger getLeftTrigger() {
    return ebdJoystick.button(9);
  }

  public Trigger getRightTrigger() {
    return ebdJoystick.button(10);
  }

  public Trigger getLeftBumper() {
    return ebdJoystick.button(5);
  }

  public Trigger getRightBumper() {
    return ebdJoystick.button(6);
  }

  public Trigger getYorTriangle() {
    return ebdJoystick.button(3);
  }

  public Trigger getBorCircle() {
    return ebdJoystick.button(1);
  }

  public Trigger getAorCross() {
    return ebdJoystick.button(2);
  }

  public Trigger getXorSquare() {
    return ebdJoystick.button(4);
  }

  public Trigger getDPadTriggerUp() {
    return ebdJoystick.povUp();
  }

  public Trigger getDPadTriggerDown() {
    return ebdJoystick.povDown();
  }

  public Trigger getDPadTriggerLeft() {
    return ebdJoystick.povLeft();
  }

  public Trigger getDPadTriggerRight() {
    return ebdJoystick.povRight();
  }
}
