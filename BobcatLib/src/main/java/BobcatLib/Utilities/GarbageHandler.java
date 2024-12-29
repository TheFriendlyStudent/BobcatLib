package BobcatLib.Utilities;

public class GarbageHandler {
  private final BobcatRunner executeCleanup;

  public GarbageHandler(int seconds) {
    executeCleanup = new BobcatRunner(() -> ExecuteCleanup(), seconds, true);
  }

  public void ExecuteCleanup() {
    System.gc();
  }

  public void stop() {
    executeCleanup.endExecution();
  }
}
