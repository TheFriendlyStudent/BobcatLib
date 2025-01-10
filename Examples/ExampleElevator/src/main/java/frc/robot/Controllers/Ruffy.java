package frc.robot.Controllers;

import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * A wrapper class for the Ruffy joystick controller that implements the {@link ControllerWrapper}
 * interface. This class provides standardized access to joystick axes and button triggers for the
 * Ruffy controller.
 */
public class Ruffy implements ControllerWrapper {
  /** The underlying joystick object for the Ruffy controller. */
  CommandJoystick ruffyJoystick;

  /** Axis index for forward/backward translation. */
  public final int translationAxis = 0;

  /** Axis index for side-to-side strafing. */
  public final int strafeAxis = 1;

  /** Axis index for rotation control. */
  public final int rotationAxis = 2;

  /**
   * Constructs a Ruffy joystick controller wrapper for the specified port.
   *
   * @param port the port the Ruffy joystick is connected to.
   */
  public Ruffy(int port) {
    ruffyJoystick = new CommandJoystick(port);
  }

  /**
   * Gets the axis value for translation (forward/backward) control.
   *
   * @return the axis value for translation control.
   */
  public double getTranslationAxis() {
    return ruffyJoystick.getRawAxis(translationAxis);
  }

  /**
   * Gets the axis value for strafing (side-to-side) control.
   *
   * @return the axis value for strafing control.
   */
  public double getStrafeAxis() {
    return ruffyJoystick.getRawAxis(strafeAxis);
  }

  /**
   * Gets the axis value for rotation control.
   *
   * @return the axis value for rotation control.
   */
  public double getRotationAxis() {
    return ruffyJoystick.getRawAxis(rotationAxis);
  }

  /**
   * Gets the trigger for the left trigger button.
   *
   * @return a {@link Trigger} object for the left trigger button.
   */
  public Trigger getLeftTrigger() {
    return new Trigger(() -> false);
  }

  /**
   * Gets the trigger for the right trigger button.
   *
   * @return a {@link Trigger} object for the right trigger button.
   */
  public Trigger getRightTrigger() {
    return new Trigger(() -> false);
  }

  /**
   * Gets the trigger for the left bumper button.
   *
   * @return a {@link Trigger} object for the left bumper button.
   */
  public Trigger getLeftBumper() {
    return new Trigger(() -> false);
  }

  /**
   * Gets the trigger for the right bumper button.
   *
   * @return a {@link Trigger} object for the right bumper button.
   */
  public Trigger getRightBumper() {
    return new Trigger(() -> false);
  }

  /**
   * Gets the trigger for the Triangle button.
   *
   * @return a {@link Trigger} object for the Triangle button.
   */
  public Trigger getYorTriangle() {
    return new Trigger(() -> false);
  }

  /**
   * Gets the trigger for the Circle button.
   *
   * @return a {@link Trigger} object for the Circle button.
   */
  public Trigger getBorCircle() {
    return new Trigger(() -> false);
  }

  /**
   * Gets the trigger for the Cross button.
   *
   * @return a {@link Trigger} object for the Cross button.
   */
  public Trigger getAorCross() {
    return new Trigger(() -> false);
  }

  /**
   * Gets the trigger for the Square button.
   *
   * @return a {@link Trigger} object for the Square button.
   */
  public Trigger getXorSquare() {
    return new Trigger(() -> false);
  }

  /**
   * Gets the trigger for the D-Pad up direction.
   *
   * @return a {@link Trigger} object for the D-Pad up direction.
   */
  public Trigger getDPadTriggerUp() {
    return new Trigger(() -> false);
  }

  /**
   * Gets the trigger for the D-Pad down direction.
   *
   * @return a {@link Trigger} object for the D-Pad down direction.
   */
  public Trigger getDPadTriggerDown() {
    return new Trigger(() -> false);
  }

  /**
   * Gets the trigger for the D-Pad left direction.
   *
   * @return a {@link Trigger} object for the D-Pad left direction.
   */
  public Trigger getDPadTriggerLeft() {
    return new Trigger(() -> false);
  }

  /**
   * Gets the trigger for the D-Pad right direction.
   *
   * @return a {@link Trigger} object for the D-Pad right direction.
   */
  public Trigger getDPadTriggerRight() {
    return new Trigger(() -> false);
  }
}
