package BobcatLib.Subsystems.Swerve.AdvancedSwerve.Util;

import edu.wpi.first.math.geometry.Rotation2d;



//                  _____  _____
//                  <     `/     |
//                   >          (
//                  |   _     _  |
//                  |  |_) | |_) |
//                  |  | \ | |   |
//                  |            |
//   ______.______%_|            |__________  _____
// _/                                       \|     |
// |                  BobcatUtil              <
// |_____.-._________              ____/|___________|
//                   | * 23-24    |
//                   |          |
//                   |            |
//                   |            |
//                   |   _        <
//                   |__/         |
//                    / `--.      |
//                  %|            |%
//              |/.%%|          -< @%%%
//              `\%`@|     v      |@@%@%%    
//            .%%%@@@|%    |    % @@@%%@%%%%
//      _.%%%%%%@@@@@@%%_/%\_%@@%%@@@@@@@%%%%%%
//   More than just a util class, it was a way of life

 public class RotationUtil {

    public static double get0to2Pi(double rad) {
        rad = rad % (2 * Math.PI);
        if (rad < (0)) {
            rad += (2 * Math.PI);
        }
        return rad;
    }
    public static double get0to2Pi(Rotation2d rot){
        return get0to2Pi(rot.getRadians());
    }
    public static double get0to360(double deg){
        deg = deg % 360;
        if (deg<0){
            deg += 360;
        }
        return deg;
    }
    public static double get0to360(Rotation2d rot){
        return get0to360(rot.getDegrees());
    }
    /**
     * wraps the rotation2d to be within one rotation,
     * i.e. a rotation2d with a value of 370 degrees will return a 
     * rotation2d with a value of 10 degrees
     */
    public static Rotation2d wrapRot2d(Rotation2d rot){
        return Rotation2d.fromRadians(get0to2Pi(rot));
    }
}
