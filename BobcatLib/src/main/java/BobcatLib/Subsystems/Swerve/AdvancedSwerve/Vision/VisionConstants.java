package BobcatLib.Subsystems.Swerve.AdvancedSwerve.Vision;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;

public class VisionConstants {
    //tags whose corresponding IDs are NOT in here will not be used in vision calculations
    //good for when a tag is wobbly or shifts over the course of a comp
    public static final int[] filtertags = {3,4,7,8,9,10,1,2,14,13};

    public static final double throwoutDist = 5.5; // meters, public static final double verticalFOV = 49.7; // degrees obviously
    public static final double rotationTolerance = 15; // maximum degrees that the limelight can be off before we throw out the pose
    public static final double zDistThreshold = 0.5; // meters that the limelight can be off the ground
    public static final int apriltagPipelineIndex = 1; //should be the same across limelights


    public static Matrix<N3, N1> trustautostdDev = VecBuilder.fill(0.2, 0.2, 9999999);
    public static Matrix<N3, N1> regautostdDev = VecBuilder.fill(0.9, 0.9, 9999999);
    public static Matrix<N3, N1> trusttelestdDev = VecBuilder.fill(0.2, 0.2, 9999999); 
    public static Matrix<N3, N1> regtelestdDev = VecBuilder.fill(0.9, 0.9, 9999999);




    public enum LimeLightType{
        LL3G_APRILTAG,
        LL3_DETECTOR
    }

    public static class LL3G{
        public static final double verticalFOV = 56.2;
        public static final double horizontalFOV = 80;
    }
    public static class LL3{
        public static final double verticalFOV = 48.9;
        public static final double horizontalFOV = 62.5;
    }

}
