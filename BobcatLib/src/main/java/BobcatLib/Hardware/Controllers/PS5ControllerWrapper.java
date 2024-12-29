package BobcatLib.Hardware.Controllers;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.button.CommandPS5Controller;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class PS5ControllerWrapper extends CommandPS5Controller implements ControllerWrapper {
  /** Controller Axis Mappings */
  public final int translationAxis = XboxController.Axis.kLeftY.value;

  public final int strafeAxis = XboxController.Axis.kLeftX.value;
  public final int rotationAxis = XboxController.Axis.kRightX.value;

  /**
   * Contains Left Joystick , Right Joystick , POV , triangle, circle, cross, square buttons.
   *
   * @param port
   */
  public PS5ControllerWrapper(int port) {
    super(port);
  }

  /**
   * Gets the axis value for translation (forward/backward) control.
   *
   * @return The axis value for translation control.
   */
  public double getTranslationAxis() {
    return super.getRawAxis(translationAxis);
  }

  /**
   * Gets the axis value for strafing (side-to-side) control.
   *
   * @return The axis value for strafing control.
   */
  public double getStrafeAxis() {
    return super.getRawAxis(strafeAxis);
  }

  /**
   * Gets the axis value for rotation control.
   *
   * @return The axis value for rotation control.
   */
  public double getRotationAxis() {
    return super.getRawAxis(rotationAxis);
  }

  public Trigger getLeftTrigger() {
    return super.L2();
  }

  public Trigger getRightTrigger() {
    return super.R2();
  }

  public Trigger getLeftBumper() {
    return super.L1();
  }

  public Trigger getRightBumper() {
    return super.R1();
  }

  public Trigger getYorTriangle() {
    return super.triangle();
  }

  public Trigger getBorCircle() {
    return super.circle();
  }

  public Trigger getAorCross() {
    return super.cross();
  }

  public Trigger getXorSquare() {
    return super.square();
  }

  public Trigger getDPadTriggerUp() {
    return super.povUp();
  }

  public Trigger getDPadTriggerDown() {
    return super.povDown();
  }

  public Trigger getDPadTriggerLeft() {
    return super.povLeft();
  }

  public Trigger getDPadTriggerRight() {
    return super.povRight();
  }
}
