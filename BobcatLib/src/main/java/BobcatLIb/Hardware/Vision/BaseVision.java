package BobcatLib.Hardware.Vision;

import BobcatLib.Hardware.Vision.Northstar.AprilTagVision;
import BobcatLib.Hardware.Vision.Northstar.AprilTagVisionIO.AprilTagVisionIOInputs;
import BobcatLib.Subsystems.Swerve.SimpleSwerve.SwerveDrive;
import BobcatLib.Utilities.Vision.VisionObservation;

public class BaseVision {

    private AprilTagVision[] northstar;
    private Limelight[] limelight;
    private SwerveDrive swerve;
    private double northstars;
    private double limelights;

    public BaseVision(SwerveDrive swerve,AprilTagVision... northstar,Limelight... limelight){
    this.swerve=swerve;
    this.limelight=limelight;
    this.northstar=northstar;
    northstars = northstar.length;
    limelights = limelight.length;    
    }
}
