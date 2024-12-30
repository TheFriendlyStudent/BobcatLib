package BobcatLib.Hardware.Controllers;

import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * Wrapper class for the Logitech controller using WPILib's CommandJoystick. This class provides
 * mappings for the controller's axes, buttons, and D-Pad.
 */
public class Logitech implements ControllerWrapper {
  private CommandJoystick logitechJoystick;

  /** Axis index for forward/backward translation. */
  public final int translationAxis = 0;

  /** Axis index for side-to-side strafing. */
  public final int strafeAxis = 1;

  /** Axis index for rotation. */
  public final int rotationAxis = 2;

  /**
   * Constructs a Logitech controller wrapper with mappings for joystick axes and buttons.
   *
   * @param port the port the controller is connected to.
   */
  public Logitech(int port) {
    logitechJoystick = new CommandJoystick(port);
  }

  /**
   * Gets the axis value for translation (forward/backward) control.
   *
   * @return the axis value for translation control.
   */
  public double getTranslationAxis() {
    return logitechJoystick.getRawAxis(translationAxis);
  }

  /**
   * Gets the axis value for strafing (side-to-side) control.
   *
   * @return the axis value for strafing control.
   */
  public double getStrafeAxis() {
    return logitechJoystick.getRawAxis(strafeAxis);
  }

  /**
   * Gets the axis value for rotation control.
   *
   * @return the axis value for rotation control.
   */
  public double getRotationAxis() {
    return logitechJoystick.getRawAxis(rotationAxis);
  }

  /**
   * Gets a placeholder trigger for the left trigger. This is a non-functional placeholder.
   *
   * @return a Trigger object for the left trigger.
   */
  public Trigger getLeftTrigger() {
    return new Trigger(() -> false);
  }

  /**
   * Gets a placeholder trigger for the right trigger. This is a non-functional placeholder.
   *
   * @return a Trigger object for the right trigger.
   */
  public Trigger getRightTrigger() {
    return new Trigger(() -> false);
  }

  /**
   * Gets the trigger for the left bumper (button 5).
   *
   * @return a Trigger object for the left bumper.
   */
  public Trigger getLeftBumper() {
    return logitechJoystick.button(5);
  }

  /**
   * Gets the trigger for the right bumper (button 6).
   *
   * @return a Trigger object for the right bumper.
   */
  public Trigger getRightBumper() {
    return logitechJoystick.button(6);
  }

  /**
   * Gets the trigger for the Y or Triangle button (button 4).
   *
   * @return a Trigger object for the Y or Triangle button.
   */
  public Trigger getYorTriangle() {
    return logitechJoystick.button(4);
  }

  /**
   * Gets the trigger for the B or Circle button (button 3).
   *
   * @return a Trigger object for the B or Circle button.
   */
  public Trigger getBorCircle() {
    return logitechJoystick.button(3);
  }

  /**
   * Gets the trigger for the A or Cross button (button 2).
   *
   * @return a Trigger object for the A or Cross button.
   */
  public Trigger getAorCross() {
    return logitechJoystick.button(2);
  }

  /**
   * Gets the trigger for the X or Square button (button 1).
   *
   * @return a Trigger object for the X or Square button.
   */
  public Trigger getXorSquare() {
    return logitechJoystick.button(1);
  }

  /**
   * Gets the trigger for the D-Pad up direction.
   *
   * @return a Trigger object for the D-Pad up direction.
   */
  public Trigger getDPadTriggerUp() {
    return logitechJoystick.povUp();
  }

  /**
   * Gets the trigger for the D-Pad down direction.
   *
   * @return a Trigger object for the D-Pad down direction.
   */
  public Trigger getDPadTriggerDown() {
    return logitechJoystick.povDown();
  }

  /**
   * Gets the trigger for the D-Pad left direction.
   *
   * @return a Trigger object for the D-Pad left direction.
   */
  public Trigger getDPadTriggerLeft() {
    return logitechJoystick.povLeft();
  }

  /**
   * Gets the trigger for the D-Pad right direction.
   *
   * @return a Trigger object for the D-Pad right direction.
   */
  public Trigger getDPadTriggerRight() {
    return logitechJoystick.povRight();
  }
}
