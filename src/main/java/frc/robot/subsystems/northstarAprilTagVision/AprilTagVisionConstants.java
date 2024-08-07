// Copyright (c) 2024 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.northstarAprilTagVision;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.util.Units;
import frc.lib.utils.LoggedTunableNumber;


public class AprilTagVisionConstants {
  public static final double ambiguityThreshold = 0.4;
  public static final double targetLogTimeSecs = 0.1;
  public static final double fieldBorderMargin = 0.5;
  public static final double zMargin = 0.75;
  public static final LoggedTunableNumber xyStdDevCoefficient = new LoggedTunableNumber("xyStdDevCoefficient", 0.005,"Cameras");
  public static final LoggedTunableNumber thetaStdDevCoefficient = new LoggedTunableNumber("thetaStdDevCoefficient", 0.01,"Cameras");;

  public static final Pose3d[] cameraPoses =
    new Pose3d[] {
        new Pose3d( // left intake side
                Units.inchesToMeters(-6.096),
                Units.inchesToMeters(-7.884),
                Units.inchesToMeters(25.413),
                new Rotation3d(0.0, Units.degreesToRadians(-10), 0.0)
                .rotateBy(new Rotation3d(0.0, 0.0, Units.degreesToRadians(180+20.0)))),
        new Pose3d(
            Units.inchesToMeters(-6.096),
            Units.inchesToMeters(7.884),
            Units.inchesToMeters(25.413),
            new Rotation3d(0.0, Units.degreesToRadians(-10), 0.0)
            .rotateBy(new Rotation3d(0.0, 0.0, Units.degreesToRadians(180-20.0)))),
            
            new Pose3d(
                Units.inchesToMeters(10.203),
                Units.inchesToMeters(-5.5422),
                Units.inchesToMeters(7.41288314),
                new Rotation3d(0.0, Units.degreesToRadians(-35), 0.0)
                .rotateBy(new Rotation3d(0.0, 0.0, Units.degreesToRadians(-10)))),
                new Pose3d(
                    Units.inchesToMeters(10.203),
                    Units.inchesToMeters(5.5422),
                    Units.inchesToMeters(7.41288314),
                    new Rotation3d(0.0, Units.degreesToRadians(-35), 0.0)
                    .rotateBy(new Rotation3d(0.0, 0.0, Units.degreesToRadians(10)))),
                
            };

  public static final String[] instanceNames = {"northstar_0", "northstar_1","northsstar_2","northstar_3"};


  public static final String[] cameraIds =   new String[] {
    "/dev/video0",
    "/dev/v4l/by-path/platform-fc880000.usb-usb-0:1:1.0-video-index0"
};

      
 
}