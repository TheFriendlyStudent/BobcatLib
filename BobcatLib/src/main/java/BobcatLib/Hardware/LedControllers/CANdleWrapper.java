package BobcatLib.Hardware.LedControllers;

import BobcatLib.Logging.Alert;
import BobcatLib.Logging.Alert.AlertType;
import BobcatLib.Logging.FaultsAndErrors.CANdleFaultsWrapper;
import BobcatLib.Utilities.CANDeviceDetails;
import com.ctre.phoenix.led.CANdle;
import com.ctre.phoenix.led.StrobeAnimation;
import edu.wpi.first.wpilibj.Timer;
import org.littletonrobotics.junction.AutoLog;

/** CANdle Wrapper */
public class CANdleWrapper implements LedControllerIO {
  /** desired behavior, untested: */
  public enum CANdleState {
    OFF,
    TURNING_CW,
    TURNING_CCW,
    RESETPOSE,
    RESETGYRO,
    ALIGNING,
    ALIGNED
  }

  /** CANdleIO Inputs */
  @AutoLog
  public static class CANdleIOInputs {
    /** State for CANdle. */
    public CANdleState state = CANdleState.OFF;
  }

  public enum BuiltInAnimations {
    ColorFlow,
    Fire,
    Larson,
    Rainbow,
    RgbFade,
    SingleFade,
    Strobe,
    Twinkle,
    TwinkleOff,
  }

  /** Is the CANdle enabled. */
  public boolean isEnabled = false;

  private CANdle leds;
  private CANdleState currState = CANdleState.OFF;
  private CANdleFaultsWrapper faults;
  private double seconds = 1;
  private Timer timer = new Timer();
  private double defaultTime = 1;
  /** Alert for Sensors. */
  public Alert sensorAlert;

  private int ledCount;

  private CANDeviceDetails details;

  public CANdleWrapper(CANDeviceDetails details, int ledCount, String canbus) {
    this.details = details;
    int id = details.getDeviceNumber();
    this.ledCount = ledCount;
    try {
      leds = new CANdle(id, canbus);
      faults = new CANdleFaultsWrapper(leds, id);
      isEnabled = true;
    } catch (Exception e) {
      // TODO: handle exception
      AlertType level = AlertType.INFO;
      sensorAlert = new Alert("LED", "LEDs " + id + " hardware not found", level);
      sensorAlert.set(true);
    }
    timer.stop();
    timer.reset();
  }

  /**
   * update the inputs for IO layer
   *
   * @param inputs CANdleIOInputs
   */
  public void updateInputs(LedControllerIOInputs inputs) {
    inputs.currState = currState;
  }

  /**
   * THIS WILL NOT AUTOMATICALLY TURN OFF, it will persist untill you set it again
   *
   * @param state the animation to play
   */
  public void setLEDs(CANdleState state) {
    timer.stop();
    timer.reset();
    if (!isEnabled) {
      return;
    }
    // Core animation logic
    if (state != currState) {
      leds.animate(null); // wipe old state when setting new one
    }
    currState = state;
    switch (state) {
      case RESETPOSE: // strobe gold
        leds.animate(new StrobeAnimation(255, 170, 0, 0, 0.25, ledCount));
        break;
      case RESETGYRO: // strobe orangish
        leds.animate(new StrobeAnimation(245, 129, 66, 0, 0.25, ledCount));
        break;
      case ALIGNING: // strobe white
        leds.animate(new StrobeAnimation(255, 255, 255, 255, 0.75, ledCount));
        break;
      case ALIGNED: // solid blue
        leds.animate(new StrobeAnimation(0, 0, 255, 0, 1, ledCount));
        break;
      case OFF:
      default:
        leds.animate(null);
        break;
    }
  }

  /**
   * @param state the animation to play
   * @param seconds duration to play
   */
  public void setLEDs(CANdleState state, double seconds) {
    setLEDs(state);
    this.seconds = seconds;
    timer.reset();
    timer.start();
  }

  /** Periodic for CANdle hardware */
  public void periodic() {
    if (timer.hasElapsed(seconds)) {
      if (isEnabled) {
        setLEDs(CANdleState.OFF);
      }
      timer.stop();
      timer.reset();
      seconds = defaultTime;
    }
  }

  public void checkForFaults() {
    faults.hasFaultOccured();
  }
}
