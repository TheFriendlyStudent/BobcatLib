package BobcatLib.Utilities;

import edu.wpi.first.wpilibj.Notifier;

public class BobcatRunner {
  private Notifier executer = null;

  /**
   * Constructs an instance of the runner, this will allow you to take an method and execute
   * periodically given a configuration you pass in.
   *
   * @param callback
   * @param delayInSeconds
   */
  public BobcatRunner(Runnable callback, int delayInSeconds, boolean isContinuous) {
    executer.setCallback(callback);
    if (isContinuous) {
      executer.startPeriodic(delayInSeconds);
    } else {
      executer.startSingle(delayInSeconds);
    }
  }

  public void endExecution() {
    executer.stop();
  }
}
