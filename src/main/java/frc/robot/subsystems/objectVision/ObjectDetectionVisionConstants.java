// Copyright (c) 2024 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.objectVision;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.util.Units;


public class ObjectDetectionVisionConstants {


  public static final Pose3d[] cameraPoses =
    new Pose3d[] {
        
      new Pose3d(
      Units.inchesToMeters(-6.38586819),
      Units.inchesToMeters(14.92279767),
      Units.inchesToMeters(26.03562025),
      new Rotation3d(0.0, Units.degreesToRadians(20), 0.0)
      .rotateBy(new Rotation3d(0.0, 0.0, Units.degreesToRadians(190)))),
              new Pose3d(
                  Units.inchesToMeters(-6.38586819),
                  Units.inchesToMeters(-14.92279767),
                  Units.inchesToMeters(26.03562025),
                  new Rotation3d(0.0, Units.degreesToRadians(20), 0.0)
                  .rotateBy(new Rotation3d(0.0, 0.0, Units.degreesToRadians(100)))),
                      
                  
                
            };

  public static final String[] instanceNames = {"ObjectDetectionVision_0"};


      
 
}