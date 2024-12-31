package BobcatLib.Subsystems.Swerve.AdvancedSwerve.Swerve.Utility.Assists;

import java.util.function.BooleanSupplier;
import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Rotation2d;

public class RotationalAssist {
    private Supplier<Rotation2d> rotationalSupplier;
    private BooleanSupplier shouldRun;
    private Supplier<Rotation2d> currRot;
    private BooleanSupplier override;


    /**
     * 
     * @param pose the pose to guide the drivetrain to
     */
    public RotationalAssist(Supplier<Rotation2d> assistRotationSupplier, Supplier<Rotation2d> rotationSupplier, BooleanSupplier shouldAssist, BooleanSupplier override){
        rotationSupplier = assistRotationSupplier;
        currRot = assistRotationSupplier;
        shouldRun = shouldAssist;
        this.override = override;
    }

    public boolean shouldAssist(){
        return shouldRun.getAsBoolean() && !override.getAsBoolean();
    }
    public Rotation2d getDesiredRot(){
        return rotationalSupplier.get();
    }
    public Rotation2d getDistanceToTarget(){
        return getDesiredRot().minus(currRot.get());
    }
    public double getErrorRad(){
        return getDistanceToTarget().getRadians();
    }


}
