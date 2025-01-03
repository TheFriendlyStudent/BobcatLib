package BobcatLib.Utilities;

import edu.wpi.first.math.geometry.Rotation2d;

public class RotationUtil {

  /**
   * @return the supplied rotation2d wrapped to be within 1 rotation, ie 361 deg becomes 1 deg, for
   *     continous wrapping
   */
  public static Rotation2d wrap(Rotation2d angle) {
    return Rotation2d.fromDegrees(((angle.getDegrees() % 360) + 360) % 360);
  }
}
