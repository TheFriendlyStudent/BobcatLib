package BobcatLib.Subsystems.Vision.Components;

import BobcatLib.Subsystems.Vision.Limelight.Estimator.PoseEstimate;
import edu.wpi.first.math.geometry.Pose2d;
import org.littletonrobotics.junction.AutoLog;

public interface VisionIO {
  @AutoLog
  public class VisionIOInputs {
    public boolean coralDetected;
    public boolean algaeDetected;
    public Pose2d pose;
  }

  public default void update() {}

  public default PoseEstimate getResult_PoseEstimate() {
    return null;
  }

  public default boolean getResult_Target() {
    return false;
  }
}
