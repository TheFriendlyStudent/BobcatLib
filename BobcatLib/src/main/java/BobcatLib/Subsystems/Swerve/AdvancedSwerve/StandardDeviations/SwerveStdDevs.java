package BobcatLib.Subsystems.Swerve.AdvancedSwerve.StandardDeviations;

import edu.wpi.first.math.Matrix;

public class SwerveStdDevs {

  public StandardDeviation autoStdDevs;
  public StandardDeviation teleStdDevs;

  public SwerveStdDevs(StandardDeviation autoStdDevs, StandardDeviation teleStdDevs) {
    this.autoStdDevs = autoStdDevs;
    this.teleStdDevs = teleStdDevs;
  }

  public Matrix[] toMatrix() {
    return new Matrix[] {
      autoStdDevs.trustMatrix(),
      teleStdDevs.trustMatrix(),
      autoStdDevs.distrustMatrix(),
      teleStdDevs.distrustMatrix()
    };
  }
}
