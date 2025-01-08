package BobcatLib.Hardware.Vision.Northstar;

import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import BobcatLib.Utilities.Vision.VisionObservation;
import BobcatLib.Utilities.Team6328.ExtensionMethod;
import BobcatLib.Utilities.Team6328.GeomUtil;
import BobcatLib.Utilities.Team6328.LoggedTunableNumber;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Quaternion;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;

import static BobcatLib.Hardware.Vision.AprilTagVisionConstants.ambiguityThreshold;
import static BobcatLib.Hardware.Vision.AprilTagVisionConstants.cameraPoses;
import static BobcatLib.Hardware.Vision.AprilTagVisionConstants.fieldBorderMargin;
import static BobcatLib.Hardware.Vision.AprilTagVisionConstants.stdDevFactors;
import static BobcatLib.Hardware.Vision.AprilTagVisionConstants.targetLogTimeSecs;
import static BobcatLib.Hardware.Vision.AprilTagVisionConstants.thetaStdDevCoefficient;
import static BobcatLib.Hardware.Vision.AprilTagVisionConstants.xyStdDevCoefficient;
import static BobcatLib.Hardware.Vision.AprilTagVisionConstants.zMargin;

import java.util.*;

import BobcatLib.Hardware.Vision.AprilTagVisionFieldConstants;
import BobcatLib.Hardware.Vision.AprilTagVisionFieldConstants.AprilTagLayoutType;
import BobcatLib.Hardware.Vision.Northstar.AprilTagVisionIO.AprilTagVisionIOInputs;
import BobcatLib.Subsystems.Swerve.SimpleSwerve.SwerveDrive;

// import org.littletonrobotics.frc2024.RobotState;
import org.littletonrobotics.junction.Logger;




/** Vision subsystem for AprilTag vision. */
@ExtensionMethod({GeomUtil.class})
public class AprilTagVision extends SubsystemBase {
  private static final LoggedTunableNumber timestampOffset =
      new LoggedTunableNumber("AprilTagVision/TimestampOffset", -(1.0 / 50.0));

  private final Supplier<AprilTagLayoutType> aprilTagTypeSupplier;
  private final AprilTagVisionIO[] io;
  private final AprilTagVisionIOInputs[] inputs;

  private final Map<Integer, Double> lastFrameTimes = new HashMap<>();
  private final Map<Integer, Double> lastTagDetectionTimes = new HashMap<>();
  private Supplier<Rotation2d> yaw;
  public VisionObservation visionPose;
  public SwerveDrive swerve;
  // private final Swerve swerve;

  public AprilTagVision(SwerveDrive swerve, Supplier<Rotation2d> yaw, Supplier<AprilTagLayoutType> aprilTagTypeSupplier, AprilTagVisionIO... io) {

    Logger.recordOutput("CamPosNT",               new Pose3d(
                  -1*Units.inchesToMeters(6.5),
                  Units.inchesToMeters(0),
                  Units.inchesToMeters(10),
                  new Rotation3d(0.0, Units.degreesToRadians(-30), Units.degreesToRadians(180.0)))
);

    this.aprilTagTypeSupplier = aprilTagTypeSupplier;
    this.io = io;
    this.yaw = yaw;
    this.swerve=swerve;
    inputs = new AprilTagVisionIOInputs[io.length];
    for (int i = 0; i < io.length; i++) {
      inputs[i] = new AprilTagVisionIOInputs();
    }

    // Create map of last frame times for instances
    for (int i = 0; i < io.length; i++) {
      lastFrameTimes.put(i, 0.0);
    }
  }

  public void periodic() {
    for (int i = 0; i < io.length; i++) {
      io[i].updateInputs(inputs[i]);
      Logger.processInputs("AprilTagVision/Inst" + i, inputs[i]);
    }


    // Loop over instances
    List<Pose2d> allRobotPoses = new ArrayList<>();
    List<Pose3d> allRobotPoses3d = new ArrayList<>();
    List<VisionObservation> allVisionObservations = new ArrayList<>();
    for (int instanceIndex = 0; instanceIndex < io.length; instanceIndex++) {
      // Loop over frames
      for (int frameIndex = 0; frameIndex < inputs[instanceIndex].timestamps.length; frameIndex++) {
        lastFrameTimes.put(instanceIndex, Timer.getFPGATimestamp());
        var timestamp = inputs[instanceIndex].timestamps[frameIndex] + timestampOffset.get();
        var values = inputs[instanceIndex].frames[frameIndex];

        // Exit if blank frame
        if (values.length == 0 || values[0] == 0) {
          continue;
        }

        // Switch based on number of poses
        Pose3d cameraPose = null;
        Pose3d robotPose3d = null;
        boolean useVisionRotation = false;
        switch ((int) values[0]) {
          case 1:
            // One pose (multi-tag), use directly
            cameraPose =
                new Pose3d(
                    values[2],
                    values[3],
                    values[4],
                    new Rotation3d(new Quaternion(values[5], values[6], values[7], values[8])));
            robotPose3d =
                cameraPose.transformBy(GeomUtil.toTransform3d(cameraPoses[instanceIndex]).inverse());
            useVisionRotation = true;
            break;
          case 2:
            // Two poses (one tag), disambiguate
            double error0 = values[1];
            double error1 = values[9];
            Pose3d cameraPose0 =
                new Pose3d(
                    values[2],
                    values[3],
                    values[4],
                    new Rotation3d(new Quaternion(values[5], values[6], values[7], values[8])));
            Pose3d cameraPose1 =
                new Pose3d(
                    values[10],
                    values[11],
                    values[12],
                    new Rotation3d(new Quaternion(values[13], values[14], values[15], values[16])));
            Pose3d robotPose3d0 =
                cameraPose0.transformBy(GeomUtil.toTransform3d(cameraPoses[instanceIndex]).inverse());
            Pose3d robotPose3d1 =
                cameraPose1.transformBy(GeomUtil.toTransform3d(cameraPoses[instanceIndex]).inverse());

            // Check for ambiguity and select based on estimated rotation
            if (error0 < error1 * ambiguityThreshold || error1 < error0 * ambiguityThreshold) {
              Rotation2d currentRotation = yaw.get();
                  
              Rotation2d visionRotation0 = robotPose3d0.toPose2d().getRotation();
              Rotation2d visionRotation1 = robotPose3d1.toPose2d().getRotation();
              if (Math.abs(currentRotation.minus(visionRotation0).getRadians())
                  < Math.abs(currentRotation.minus(visionRotation1).getRadians())) {
                cameraPose = cameraPose0;
                robotPose3d = robotPose3d0;
              } else {
                cameraPose = cameraPose1;
                robotPose3d = robotPose3d1;
              }
            }
            break;
        }

        Logger.recordOutput("Northstar", robotPose3d);

        // Exit if no data
        if (cameraPose == null || robotPose3d == null) {
          continue;
        }

        // Exit if robot pose is off the field
        if (robotPose3d.getX() < -fieldBorderMargin
            || robotPose3d.getX() > AprilTagVisionFieldConstants.fieldLength + fieldBorderMargin
            || robotPose3d.getY() < -fieldBorderMargin
            || robotPose3d.getY() > AprilTagVisionFieldConstants.fieldWidth + fieldBorderMargin
            || robotPose3d.getZ() < -zMargin
            || robotPose3d.getZ() > zMargin) {
          continue;
        }

        // Get 2D robot pose
        Pose2d robotPose = robotPose3d.toPose2d();

        // Get tag poses and update last detection times
        List<Pose3d> tagPoses = new ArrayList<>();
        for (int i = (values[0] == 1 ? 9 : 17); i < values.length; i++) {
          int tagId = (int) values[i];
          lastTagDetectionTimes.put(tagId, Timer.getFPGATimestamp());
          Optional<Pose3d> tagPose =
              aprilTagTypeSupplier.get().getLayout().getTagPose((int) values[i]);
          tagPose.ifPresent(tagPoses::add);
        }
        if (tagPoses.size() == 0) continue;

        // Calculate average distance to tag
        double totalDistance = 0.0;
        for (Pose3d tagPose : tagPoses) {
          totalDistance += tagPose.getTranslation().getDistance(cameraPose.getTranslation());
        }
        double avgDistance = totalDistance / tagPoses.size();

        // Add observation to list
        double xyStdDev =
            xyStdDevCoefficient
                * Math.pow(avgDistance, 2.0)
                / tagPoses.size()
                * stdDevFactors[instanceIndex];
        double thetaStdDev =
            useVisionRotation
                ? thetaStdDevCoefficient
                    * Math.pow(avgDistance, 2.0)
                    / tagPoses.size()
                    * stdDevFactors[instanceIndex]
                : Double.POSITIVE_INFINITY;
        allVisionObservations.add(
            new VisionObservation(
                robotPose, timestamp, VecBuilder.fill(xyStdDev, xyStdDev, thetaStdDev)));
        allRobotPoses.add(robotPose);
        allRobotPoses3d.add(robotPose3d);

        swerve.addVision(new VisionObservation(robotPose, timestamp, VecBuilder.fill(xyStdDev, xyStdDev, thetaStdDev)));
        // visionPose=new VisionObservation(robotPose, timestamp, VecBuilder.fill(xyStdDev, xyStdDev, thetaStdDev));

        // Log data from instance
        Logger.recordOutput(
            "AprilTagVision/Inst" + instanceIndex + "/LatencySecs",
            Timer.getFPGATimestamp() - timestamp);
        Logger.recordOutput("AprilTagVision/Inst" + instanceIndex + "/RobotPose", robotPose);
        Logger.recordOutput("AprilTagVision/Inst" + instanceIndex + "/RobotPose3d", robotPose3d);
        Logger.recordOutput(
            "AprilTagVision/Inst" + instanceIndex + "/TagPoses", tagPoses.toArray(Pose3d[]::new));
      }

      // If no frames from instances, clear robot pose
      if (inputs[instanceIndex].timestamps.length == 0) {
        Logger.recordOutput("AprilTagVision/Inst" + instanceIndex + "/RobotPose", new Pose2d());
        Logger.recordOutput("AprilTagVision/Inst" + instanceIndex + "/RobotPose3d", new Pose3d());
      }

      // If no recent frames from instance, clear tag poses
      if (Timer.getFPGATimestamp() - lastFrameTimes.get(instanceIndex) > targetLogTimeSecs) {
        //noinspection RedundantArrayCreation
        Logger.recordOutput("AprilTagVision/Inst" + instanceIndex + "/TagPoses", new Pose3d[] {});
      }
    }

    // Log robot poses
    Logger.recordOutput("AprilTagVision/RobotPoses", allRobotPoses.toArray(Pose2d[]::new));
    Logger.recordOutput("AprilTagVision/RobotPoses3d", allRobotPoses3d.toArray(Pose3d[]::new));

    // Log tag poses
    List<Pose3d> allTagPoses = new ArrayList<>();
    for (Map.Entry<Integer, Double> detectionEntry : lastTagDetectionTimes.entrySet()) {
      if (Timer.getFPGATimestamp() - detectionEntry.getValue() < targetLogTimeSecs) {
        aprilTagTypeSupplier
            .get()
            .getLayout()
            .getTagPose(detectionEntry.getKey())
            .ifPresent(allTagPoses::add);
      }
    }
    Logger.recordOutput("AprilTagVision/TagPoses", allTagPoses.toArray(Pose3d[]::new));

    // Send results to robot state
    // allVisionObservations.stream()
    //     .sorted(Comparator.comparingDouble(VisionObservation::timestamp))
    //     .forEach(RobotState.getInstance()::addVisionObservation);
  }
}
