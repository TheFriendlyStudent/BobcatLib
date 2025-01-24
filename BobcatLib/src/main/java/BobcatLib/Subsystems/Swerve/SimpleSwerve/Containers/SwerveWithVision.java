package BobcatLib.Subsystems.Swerve.SimpleSwerve.Containers;

import static edu.wpi.first.units.Units.DegreesPerSecond;

import BobcatLib.Hardware.Controllers.OI;
import BobcatLib.Subsystems.Swerve.SimpleSwerve.Swerve.Module.Utility.PIDConstants;
import BobcatLib.Subsystems.Swerve.SimpleSwerve.Swerve.Module.Utility.Pose.WpiPoseEstimator;
import BobcatLib.Subsystems.Swerve.SimpleSwerve.Utility.Alliance;
import BobcatLib.Subsystems.Swerve.Utility.LoadablePathPlannerAuto;
import BobcatLib.Subsystems.Vision.Components.VisionIO.target;
import BobcatLib.Subsystems.Vision.Components.VisionPoseDetector;
import BobcatLib.Subsystems.Vision.Limelight.LimeLightConfig;
import BobcatLib.Subsystems.Vision.Limelight.Structures.AngularVelocity3d;
import BobcatLib.Subsystems.Vision.Limelight.Structures.Orientation3d;
import BobcatLib.Subsystems.Vision.VisionSubsystem;
import edu.wpi.first.math.geometry.Rotation3d;
import java.util.List;

public class SwerveWithVision extends SwerveBase {

  public final VisionSubsystem limelightVision;

  public SwerveWithVision(
      OI driver_controller,
      List<LoadablePathPlannerAuto> autos,
      String robotName,
      boolean isSim,
      Alliance alliance,
      PIDConstants tranPidPathPlanner,
      PIDConstants rotPidPathPlanner,
      String VisionName,
      List<target> targets,
      LimeLightConfig ll_cfg) {
    super(
        driver_controller,
        autos,
        robotName,
        isSim,
        alliance,
        tranPidPathPlanner,
        rotPidPathPlanner);

    // Sets up the initial states of the robot orientation
    Rotation3d rot3d = new Rotation3d(s_Swerve.getGyroYaw());
    Orientation3d orientation =
        new Orientation3d(
            rot3d,
            new AngularVelocity3d(
                DegreesPerSecond.of(0), DegreesPerSecond.of(0), DegreesPerSecond.of(0)));

    // Initialize the limelight Vision Subystem !
    limelightVision =
        new VisionSubsystem(
            VisionName,
            new VisionPoseDetector(
                "VisionDetector",
                targets,
                orientation,
                (WpiPoseEstimator) s_Swerve.swerveDrivePoseEstimator,
                ll_cfg));
  }

  public void periodic() {
    limelightVision.updatePoseEstimator(s_Swerve.getGyroYaw(), s_Swerve.getModulePositions());
    limelightVision.periodic();
  }
}
