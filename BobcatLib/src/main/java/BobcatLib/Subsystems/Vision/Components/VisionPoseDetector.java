package BobcatLib.Subsystems.Vision.Components;

import BobcatLib.Subsystems.Swerve.SimpleSwerve.Swerve.Module.Utility.Pose.WpiPoseEstimator;
import BobcatLib.Subsystems.Vision.Limelight.Estimator.LimelightPoseEstimator;
import BobcatLib.Subsystems.Vision.Limelight.Estimator.PoseEstimate;
import BobcatLib.Subsystems.Vision.Limelight.LimeLightConfig;
import BobcatLib.Subsystems.Vision.Limelight.LimelightCamera;
import BobcatLib.Subsystems.Vision.Limelight.Structures.LimelightSettings.LEDMode;
import BobcatLib.Subsystems.Vision.Limelight.Structures.Orientation3d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.util.Units;
import java.util.List;
import java.util.Optional;

public class VisionPoseDetector implements VisionIO {

  private final String name;
  private boolean isFound;
  private List<target> targets;
  private LimelightCamera limelight;
  private LimelightPoseEstimator poseEstimator;
  private WpiPoseEstimator wpiPoseEstimator;
  private LimeLightConfig cfg;

  public final double verticalFOV = 49.7; // Degrees
  public final double horizontalFOV = 63.3; // Degrees
  public final double limelightMountHeight = Units.inchesToMeters(20.5); // meters
  public final int horPixles = 1280; // Pixles
  public final int verPixles = 640; // Pixles

  public VisionPoseDetector(
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

    poseEstimator = limelight.getPoseEstimator(true);
  }

  public void updateInputs(VisionIOInputs inputs) {
    inputs.algaeDetected = false;
    inputs.coralDetected = false;
    inputs.pose = getResult_PoseEstimate().pose.toPose2d();
  }

  public void update() {}

  public target findInstanceOf(String objectType) {
    target t =
        targets.stream().filter(target -> target.name == objectType).findFirst().orElse(null);
    return t;
  }

  public void updatePoseEstimator(WpiPoseEstimator wpi) {
    wpiPoseEstimator = wpi;
  }

  public void updatePoseEstimator(Rotation2d angle, SwerveModulePosition[] positions) {
    wpiPoseEstimator.swerveDrivePoseEstimator.update(angle, positions);
  }

  /**
   * Input Tab - Change "Pipeline Type" to "Fiducial Markers" Input Tab - Use the highest available
   * resolution for 3D tracking, or use 640x480 for pure 2D tracking. Standard Tab - Make sure
   * "family" is set to "AprilTag Classic 36h11" Input Tab - Set "Black Level" to zero Input Tab -
   * Set "Gain" to 15 Input Tab - Reduce exposure to reduce tracking loss while in motion due to
   * motion blur. Stop reducing once tracking reliability decreases. You may need to increase
   * exposure at dimly-lit events. Standard Tab - If would like to increase your framerate, increase
   * the "Detector Downscale" If you want to use 3D tracking and 2D tx/ty tracking simultaneously,
   * set the priority Tag ID over networktables to configure the preferred tag for 2D tracking.
   * Click the "Gear" Icon, and make sure your team number is set and that a static IP is
   * configured. Click "Change Team Number" and "Change IP Settings" if you changed their
   * corresponding settings. Powercycle your robot. You're done! Use "tx" and "ty" from
   * networktables. Copy the code sample on the "getting started" page.
   *
   * <p>Gets the Pose Estimate given any seen apriltags!
   */
  public PoseEstimate getResult_PoseEstimate() {
    // Alternatively you can do
    Optional<PoseEstimate> visionEstimate = limelight.getPoseEstimator(true).getPoseEstimate();
    // If the pose is present
    visionEstimate.ifPresent(
        (PoseEstimate poseEstimate) -> {
          if (poseEstimate.avgTagDist < cfg.tagDistanceLimit
              && poseEstimate.tagCount > 0
              && poseEstimate.getMinTagAmbiguity() < cfg.tagAmbiguity) {
            // Add it to the pose estimator.
            wpiPoseEstimator.swerveDrivePoseEstimator.addVisionMeasurement(
                poseEstimate.pose.toPose2d(), poseEstimate.timestampSeconds);
          }
        });
    return visionEstimate.get();
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
