package BobcatLib.Utilities;

public class GarbageHandler {
  private final Runner executeCleanup;

  public GarbageHandler(int seconds) {
    executeCleanup = new Runner(() -> ExecuteCleanup(), seconds, true);
  }

  public void ExecuteCleanup() {
    System.gc();
  }

  public void stop() {
    executeCleanup.endExecution();
  }
}
