package BobcatLib.Subsystems.Vision.Components;

import BobcatLib.Subsystems.Swerve.SimpleSwerve.Swerve.Module.Utility.Pose.WpiPoseEstimator;
import BobcatLib.Subsystems.Vision.Limelight.Estimator.PoseEstimate;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import org.littletonrobotics.junction.AutoLog;

public interface VisionIO {
  public class target {
    public boolean isSeen = false;
    public String name = "";
    public double distance;
  }

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

  public default void updatePoseEstimator(WpiPoseEstimator wpi) {}

  public default void updatePoseEstimator(Rotation2d angle, SwerveModulePosition[] positions) {}

  public default target findInstanceOf(String objectType) {
    return null;
  }
}
