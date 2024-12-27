package BobcatLib.Hardware.Controllers;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class XboxControllerWrapper extends CommandXboxController implements ControllerWrapper {
  /** Controller Axis Mappings */
  public final int translationAxis = XboxController.Axis.kLeftY.value;

  public final int strafeAxis = XboxController.Axis.kLeftX.value;
  public final int rotationAxis = XboxController.Axis.kRightX.value;

  /**
   * Contains Left Joystick , Right Joystick , POV , y , b , a ,x buttons.
   *
   * @param port
   */
  public XboxControllerWrapper(int port) {
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
    return super.leftTrigger(0.3);
  }

  public Trigger getRightTrigger() {
    return super.rightTrigger(0.3);
  }

  public Trigger getLeftBumper() {
    return super.leftBumper();
  }

  public Trigger getRightBumper() {
    return super.rightBumper();
  }

  public Trigger getYorTriangle() {
    return super.y();
  }

  public Trigger getBorCircle() {
    return super.b();
  }

  public Trigger getAorCross() {
    return super.a();
  }

  public Trigger getXorSquare() {
    return super.x();
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
