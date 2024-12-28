// Copyright (c) 2024 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package BobcatLib.Hardware.Vision.Northstar;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.util.Units;

public class AprilTagVisionConstants {
  public static final double ambiguityThreshold = 0.4;
  public static final double targetLogTimeSecs = 0.1;
  public static final double fieldBorderMargin = 0.5;
  public static final double zMargin = 0.75;
  public static final double xyStdDevCoefficient = 0.005;
  public static final double thetaStdDevCoefficient = 0.01;

  public static final double[] stdDevFactors =new double[] {1.0, 1.0};

  public static final Pose3d[] cameraPoses =
            new Pose3d[] {
              new Pose3d(
                  -1*Units.inchesToMeters(6.5),
                  Units.inchesToMeters(0),
                  Units.inchesToMeters(10),
                  new Rotation3d(0.0, Units.degreesToRadians(-30), Units.degreesToRadians(180.0))),
                      // .rotateBy(new Rotation3d(0.0, 0.0, Units.degreesToRadians(30.0)))),
              new Pose3d(
                  Units.inchesToMeters(0),
                  Units.inchesToMeters(0),
                  Units.inchesToMeters(0),
                  new Rotation3d(0.0, Units.degreesToRadians(0), 0.0)
                      .rotateBy(new Rotation3d(0.0, 0.0, Units.degreesToRadians(0.0))))
            };

  public static final String[] instanceNames =
        new String[] {"northstar0", "northstar1"};

  public static final String[] cameraIds =
            new String[] {
              "/dev/video0",
              "cam1"
            };
}
