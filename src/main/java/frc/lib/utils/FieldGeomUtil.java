package frc.lib.utils;

import java.util.ArrayList;
import java.util.HashMap;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.Constants;
import frc.robot.Constants.FieldConstants;

public class FieldGeomUtil {
  Alliance m_allianceColor;

  public FieldGeomUtil() {
    m_allianceColor = DriverStation.getAlliance().get();


  }

  

  public Pose2d flipSidePose2d(Pose2d startPose) {

    return new Pose2d(startPose.getX(),
        FieldConstants.kFieldWidthMeters - startPose.getY(),
        startPose.getRotation());
  }

  public Pose3d flipSidePose3d(Pose3d startPose) {
    return new Pose3d(startPose.getX(),
        FieldConstants.kFieldWidthMeters - startPose.getY(), startPose.getZ(),
        startPose.getRotation());
  }

}
