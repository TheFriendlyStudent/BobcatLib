package BobcatLib.Subsystems.Vision.Components;

import BobcatLib.Subsystems.Swerve.SimpleSwerve.Swerve.Module.Utility.Pose.WpiPoseEstimator;
import BobcatLib.Subsystems.Vision.Limelight.LimeLightConfig;
import BobcatLib.Subsystems.Vision.Limelight.LimelightCamera;
import BobcatLib.Subsystems.Vision.Limelight.Structures.LimelightResults;
import BobcatLib.Subsystems.Vision.Limelight.Structures.LimelightSettings.LEDMode;
import BobcatLib.Subsystems.Vision.Limelight.Structures.Orientation3d;
import BobcatLib.Subsystems.Vision.Limelight.Structures.Target.Pipeline.NeuralClassifier;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.util.Units;
import java.util.List;

public class TargetDetector implements VisionIO {

  private final String name;
  private boolean isFound;
  private List<target> targets;
  private LimelightCamera limelight;
  private LimeLightConfig cfg;

  public final double verticalFOV = 49.7; // Degrees
  public final double horizontalFOV = 63.3; // Degrees
  public final double limelightMountHeight = Units.inchesToMeters(20.5); // meters
  public final int horPixles = 1280; // Pixles
  public final int verPixles = 640; // Pixles

  public TargetDetector(
      String name,
      List<target> targets,
      Orientation3d orientation,
      WpiPoseEstimator swerveDrivePoseEstimator,
      LimeLightConfig cfg) {
    this.name = name;
    this.targets = targets;
    limelight = new LimelightCamera(name);
    this.cfg = cfg;

    limelight
        .getSettings()
        .withLimelightLEDMode(LEDMode.PipelineControl)
        .withCameraOffset(Pose3d.kZero)
        .withRobotOrientation(orientation)
        .save();
  }

  public void updateInputs(VisionIOInputs inputs) {
    inputs.algaeDetected = findInstanceOf("algae").isSeen;
    inputs.coralDetected = findInstanceOf("coral").isSeen;
    inputs.pose = getResult_PoseEstimate().pose.toPose2d();
  }

  public void update() {}

  public target findInstanceOf(String objectType) {
    target t =
        targets.stream().filter(target -> target.name == objectType).findFirst().orElse(null);
    return t;
  }

  public void updatePoseEstimator(WpiPoseEstimator wpi) {}

  public void updatePoseEstimator(Rotation2d angle, SwerveModulePosition[] positions) {}

  public boolean getResult_Target() {
    // Get the results
    limelight
        .getLatestResults()
        .ifPresent(
            (LimelightResults result) -> {
              for (NeuralClassifier object : result.targets_Classifier) {
                for (target t : targets) {
                  if (object.className.equals(t.name)) {
                    t.isSeen = false;
                    // Check pixel location
                    if (object.ty > 2 && object.ty < 1) {
                      // Target is valid! do stuff!
                      isFound = true;
                      t.isSeen = true;
                      double diameter = 0;
                      if (t.name == "algae") {
                        diameter = 18;
                      } else {
                        diameter = 3;
                      }
                      t.distance =
                          distanceFromCameraPercentage(
                              object.zone, diameter, limelightMountHeight, true);
                    }
                  }
                }
              }
            });
    return isFound;
  }

  /**
   * @param widthPercent [0,1], percentage of the vertical width of the image that the note is
   *     taking up
   * @return distance in meters
   */
  public double distanceFromCameraPercentage(
      double widthPercent, double diameter, double limelightMountHeight, boolean tv) {
    if (tv) {
      widthPercent = pixlesToPercent(widthPercent);
      double hypotDist = ((180 * diameter) / (63.3 * Math.PI)) * (1 / widthPercent);
      double intakeDist =
          Math.sqrt(
              (hypotDist * hypotDist) - (limelightMountHeight * limelightMountHeight)); // distance
      // to
      // intake
      return intakeDist;
    } else {
      return 0;
    }
  }

  public double pixlesToPercent(double pixels) {
    return pixels / horPixles;
  }
}
