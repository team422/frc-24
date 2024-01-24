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
// import frc.lib.pathplanner.ExtendedPathPoint;
import frc.robot.Constants;
import frc.robot.Constants.FieldConstants;

public class FieldGeomUtil {
  // HashMap<String, ExtendedPathPoint> allPoints = new HashMap<>();
  // public HashMap<String, ExtendedPathPoint> allGamePieces = new HashMap<>();
  public HashMap<String, Pose3d> allNodes = new HashMap<>();

  Alliance m_allianceColor;

  public FieldGeomUtil() {
    m_allianceColor = DriverStation.getAlliance().get();

    // allPoints.put("blueLeftWallLoadingStation", Constants.Setpoints.blueLeftWallLoadingStation);


    allNodes.put("blueFirstGridLeftMid", new Pose3d(0.86, 0.50, 1, new Rotation3d()));
    allNodes.put("blueFirstGridLeftHigh", new Pose3d(0.45, 0.50, 1.3, new Rotation3d()));
    allNodes.put("blueFirstGridCubeHigh", new Pose3d(0.45, 1.08, 1.5, new Rotation3d()));
    allNodes.put("blueFirstGridCubeMid", new Pose3d(0.47, 1.08, .98, new Rotation3d()));
    allNodes.put("blueFirstGridRightMid", new Pose3d(.86, 1.64, 1, new Rotation3d()));
    allNodes.put("blueFirstGridRightHigh", new Pose3d(0.45, 1.64, 1.3, new Rotation3d()));

    allNodes.put("blueSecondGridLeftMid", new Pose3d(0.86, 2.20, 1, new Rotation3d()));
    allNodes.put("blueSecondGridLeftHigh", new Pose3d(0.45, 2.20, 1.3, new Rotation3d()));
    allNodes.put("blueSecondGridCubeHigh", new Pose3d(0.45, 2.78, 1.5, new Rotation3d()));
    allNodes.put("blueSecondGridCubeMid", new Pose3d(0.47, 2.78, .98, new Rotation3d()));
    allNodes.put("blueSecondGridRightMid", new Pose3d(.86, 3.34, 1, new Rotation3d()));
    allNodes.put("blueSecondGridRightHigh", new Pose3d(0.45, 3.34, 1.3, new Rotation3d()));

    allNodes.put("blueThirdGridLeftMid", new Pose3d(0.86, 3.90, 1, new Rotation3d()));
    allNodes.put("blueThirdGridLeftHigh", new Pose3d(0.45, 3.90, 1.3, new Rotation3d()));
    allNodes.put("blueThirdGridCubeHigh", new Pose3d(0.45, 4.48, 1.5, new Rotation3d()));
    allNodes.put("blueThirdGridCubeMid", new Pose3d(0.47, 4.48, .98, new Rotation3d()));
    allNodes.put("blueThirdGridRightMid", new Pose3d(.86, 5.04, 1, new Rotation3d()));
    allNodes.put("blueThirdGridRightHigh", new Pose3d(0.45, 5.04, 1.3, new Rotation3d()));

    // Logger.getInstance().recordOutput("blueFirstGridLeftMid", allNodes.get("blueFirstGridLeftMid"));
    // Logger.getInstance().recordOutput("blueFirstGridLeftHigh", allNodes.get("blueFirstGridLeftHigh"));
    // Logger.getInstance().recordOutput("blueSecondGridLeftMid", allNodes.get("blueSecondGridLeftMid"));
    // Logger.getInstance().recordOutput("blueSecondGridLeftHigh", allNodes.get("blueSecondGridLeftHigh"));
    // Logger.getInstance().recordOutput("blueSecondCubeHigh", allNodes.get("blueSecondGridCubeHigh"));
    // Logger.getInstance().recordOutput("blueSecondCubeMid", allNodes.get("blueSecondGridCubeMid"));
    // Logger.getInstance().recordOutput("blueSecondGridRightMid", allNodes.get("blueSecondGridRightMid"));
    // Logger.getInstance().recordOutput("blueSecondGridRightHigh", allNodes.get("blueSecondGridRightHigh"));
    // Logger.getInstance().recordOutput("blueThirdGridLeftMid", allNodes.get("blueThirdGridLeftMid"));
    // Logger.getInstance().recordOutput("blueThirdGridLeftHigh", allNodes.get("blueThirdGridLeftHigh"));
    // Logger.getInstance().recordOutput("blueThirdCubeHigh", allNodes.get("blueThirdGridCubeHigh"));
    // Logger.getInstance().recordOutput("blueThirdCubeMid", allNodes.get("blueThirdGridCubeMid"));
    // Logger.getInstance().recordOutput("blueThirdGridRightMid", allNodes.get("blueThirdGridRightMid"));
    // Logger.getInstance().recordOutput("blueThirdGridRightHigh", allNodes.get("blueThirdGridRightHigh"));

    // HashMap<String, ExtendedPathPoint> redSide = new HashMap<String, ExtendedPathPoint>();
    // redSide.put("redLeftWallLoadingStation", Constants.SetpointConstants.redLeftWallLoadingStation);
    // redSide.put("redRightWallLoadingStation", Constants.SetpointConstants.redRightWallLoadingStation);
    // redSide.put("redFirstGridLeftCone", Constants.SetpointConstants.redFirstGridLeftCone);
    // redSide.put("redFirstGridCube", Constants.SetpointConstants.redFirstGridCube);
    // redSide.put("redFirstGridRightCone", Constants.SetpointConstants.redFirstGridRightCone);
    // redSide.put("redSecondGridLeftCone", Constants.SetpointConstants.redSecondGridLeftCone);
    // redSide.put("redSecondGridCube", Constants.SetpointConstants.redSecondGridCube);
    // redSide.put("redSecondGridRightCone", Constants.SetpointConstants.redSecondGridRightCone);
    // redSide.put("redThirdGridLeftCone", Constants.SetpointConstants.redThirdGridLeftCone);
    // redSide.put("redThirdGridCube", Constants.SetpointConstants.redThirdGridCube);
    // redSide.put("redThirdGridRightCone", Constants.SetpointConstants.redThirdGridRightCone);
    // redSide.put("redLeftOfBalance", Constants.SetpointConstants.redLeftOfBalance);
    // redSide.put("redRightOfBalance", Constants.SetpointConstants.redRightOfBalance);
    // redSide.put("redPreLoadingStation", Constants.SetpointConstants.redPreLoadingStation);

    // allPoints.put("blue", blueSide);
    // allPoints.put("red", redSide);

  }

  public Pose3d getScoringPose(String scoringPoseName) {
    if (DriverStation.getAlliance().get() == Alliance.Red) {
      return flipSidePose3d(allNodes.get(scoringPoseName));
    }
    return allNodes.get(scoringPoseName);
  }

  public Pose3d getClosestScoringPose(Pose2d curPose, int m_height) {
    Pose3d closestPose = null;
    Pose2d projectedPose = new Pose2d();
    double closestDistance = Double.MAX_VALUE;
    if (m_height == 1) {
      projectedPose = new Pose2d(1,
          curPose.getY() + (curPose.getX() - 1) * Rotation2d.fromDegrees(180).minus(curPose.getRotation()).getSin(),
          curPose.getRotation());
    }
    if (m_height == 2) {
      projectedPose = new Pose2d(0.86,
          curPose.getY() + (curPose.getX() - .86) * Rotation2d.fromDegrees(180).minus(curPose.getRotation()).getSin(),
          curPose.getRotation());
    }
    if (m_height == 3) {
      projectedPose = new Pose2d(0.46,
          curPose.getY() + (curPose.getX() - .46) * Rotation2d.fromDegrees(180).minus(curPose.getRotation()).getSin(),
          curPose.getRotation());
    }
    if (DriverStation.getAlliance().get() == Alliance.Red) {
      projectedPose = flipSidePose2d(projectedPose);
    }

    Logger.getInstance().recordOutput("projected pose", projectedPose);

    for (String poseName : allNodes.keySet()) {
      Pose2d desPose = allNodes.get(poseName).toPose2d();
      if (poseName.contains("High") && m_height != 3) {
        continue;
      }
      if (poseName.contains("Mid") && m_height != 2) {
        continue;
      }
      if (poseName.contains("Low") && m_height != 1) {
        continue;
      }
      // project the current pose onto the y axis of the desired pose based on the angle of the desired pose

      double distance = Math.abs(projectedPose.getY() - desPose.getY());
      if (distance < closestDistance) {
        closestDistance = distance;
        closestPose = allNodes.get(poseName);
      }
    }
    if (DriverStation.getAlliance().get() == Alliance.Red) {
      return flipSidePose3d(closestPose);
    }
    return closestPose;
  }

  public boolean overConesOrCubes(Pose3d pose) {
    // return false;
    if ((pose.getX() < 1 || pose.getX() > 15.5) && pose.getY() < 6) {
      return true;
    }
    return false;
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
