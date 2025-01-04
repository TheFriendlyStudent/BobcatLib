package BobcatLib.Subsystems.Swerve.AdvancedSwerve.Assists;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import java.util.function.BooleanSupplier;
import java.util.function.Supplier;

public class AutoAlign {
  Supplier<Rotation2d> goalSupplier;
  BooleanSupplier activate;
  public PIDController pidTheta;

  /**
   * pid units in radians
   *
   * @param goalSupplier
   * @param activate
   * @param kp
   * @param ki
   * @param kd
   */
  public AutoAlign(
      Supplier<Rotation2d> goalSupplier,
      BooleanSupplier activate,
      double kp,
      double ki,
      double kd) {

    this.goalSupplier = goalSupplier;
    this.activate = activate;

    pidTheta = new PIDController(kp, ki, kd);
    // wrap values, 0 and 2pi are considered the same point, so the pid can
    // calculate paths that go through them in a circle
    // inputs must be in radians!
    pidTheta.enableContinuousInput(0, 2 * Math.PI);
  }

  public Rotation2d distanceToTarget(Rotation2d currentYaw) {
    return currentYaw.minus(goalSupplier.get());
  }

  public double output(Rotation2d currentYaw) {
    return pidTheta.calculate(goalSupplier.get().getRadians(), currentYaw.getRadians());
  }

  public boolean active() {
    return activate.getAsBoolean();
  }
}
