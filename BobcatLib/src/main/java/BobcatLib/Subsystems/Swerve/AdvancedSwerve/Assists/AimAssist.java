package BobcatLib.Subsystems.Swerve.AdvancedSwerve.Assists;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import java.util.function.BooleanSupplier;
import java.util.function.Supplier;

public class AimAssist {
  Supplier<Translation2d> goalSupplier;
  BooleanSupplier activate;
  public PIDController pidX;
  public PIDController pidY;

  public AimAssist(
      Supplier<Translation2d> goalSupplier,
      BooleanSupplier activate,
      double kp,
      double ki,
      double kd) {
    this(goalSupplier, activate, kp, ki, kd, kp, ki, kd);
  }

  public AimAssist(
      Supplier<Translation2d> goalSupplier,
      BooleanSupplier activate,
      double kpx,
      double kix,
      double kdx,
      double kpy,
      double kiy,
      double kdy) {

    this.goalSupplier = goalSupplier;
    this.activate = activate;

    pidX = new PIDController(kpx, kix, kdx);
    pidY = new PIDController(kpy, kiy, kdy);
  }

  public double distanceToTarget(Translation2d pose) {
    return pose.getDistance(goalSupplier.get());
  }

  public double outputX(Translation2d pose) {
    return pidX.calculate(goalSupplier.get().getX(), pose.getX());
  }

  public double outputY(Translation2d pose) {
    return pidX.calculate(goalSupplier.get().getY(), pose.getY());
  }

  public boolean active() {
    return activate.getAsBoolean();
  }
}
