// Copyright (c) 2024 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.
package frc.robot.subsystems.northstarAprilTagVision;

import java.util.ArrayList;
import java.util.Comparator;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.Optional;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Quaternion;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.Timer;
import frc.lib.advantagekit.LoggerUtil;
import frc.lib.utils.VirtualSubsystem;
import frc.robot.Constants;
import frc.robot.FieldConstants;
import frc.robot.RobotState;
import frc.robot.RobotState.VisionObservation;
import frc.robot.subsystems.northstarAprilTagVision.AprilTagVisionIO.AprilTagVisionIOInputs;
import frc.robot.utils.GeomUtil;

import static frc.robot.subsystems.northstarAprilTagVision.AprilTagVisionConstants.*;

/** Vision subsystem for AprilTag vision. */
public class AprilTagVision extends VirtualSubsystem {

  private final AprilTagVisionIO[] io;
  private final AprilTagVisionIOInputs[] inputs;

  private final Map<Integer, Double> lastFrameTimes = new HashMap<>();
  private final Map<Integer, Double> lastTagDetectionTimes = new HashMap<>();
  private final ArrayList<Pose2d> averageStoppedPositions = new ArrayList<>();

  public AprilTagVision(AprilTagVisionIO... io) {
    this.io = io;
    inputs = new AprilTagVisionIOInputs[io.length];
    for (int i = 0; i < io.length; i++) {
      inputs[i] = new AprilTagVisionIOInputs();
    }

    // Create map of last frame times for instances
    for (int i = 0; i < io.length; i++) {
      lastFrameTimes.put(i, 0.0);
    }

    // Create map of last detection times for tags
    FieldConstants.getAprilTags()
        .getTags()
        .forEach(
            (AprilTag tag) -> {
              lastTagDetectionTimes.put(tag.ID, 0.0);
            });
  }

  public void periodic() {
    
    for (int i = 0; i < io.length; i++) {
      
      io[i].updateInputs(inputs[i]);
      
      Logger.processInputs("AprilTagVision/Inst" + i, inputs[i]);
    }

    // Loop over instances
    List<Pose2d> allRobotPoses = new ArrayList<>();
    List<Pose3d> allRobotPoses3d = new ArrayList<>();
    List<VisionObservation> allVisionObservations = new ArrayList<>();
    int total = 0;
    for (int instanceIndex = 0; instanceIndex < io.length; instanceIndex++) {
      // Loop over frames
      if(edu.wpi.first.wpilibj.RobotState.isAutonomous()){
      total += inputs[instanceIndex].timestamps.length;
      if(total > 25){
        return; 
      }
    }
      for (int frameIndex = 0; frameIndex < inputs[instanceIndex].timestamps.length; frameIndex++) {
        lastFrameTimes.put(instanceIndex, Timer.getFPGATimestamp());
        var timestamp = inputs[instanceIndex].timestamps[frameIndex];
        if (Constants.getMode() == Constants.Mode.REPLAY) {
          timestamp = Timer.getFPGATimestamp() -0.1;
        }
        // subtract out the frame time
        if (inputs[instanceIndex].fps > 0){
          timestamp -=  1/inputs[instanceIndex].fps;
        }
        var values = inputs[instanceIndex].frames[frameIndex];

        // Exit if blank frame
        if (values.length == 0 || values[0] == 0) {
          continue;
        }
        // System.out.println(System.currentTimeMillis());
        // Switch based on number of poses
        Pose3d cameraPose = null;
        Pose3d robotPose3d = null;
        boolean useVisionRotation = false;
        switch ((int) values[0]) {
          case 1:
            // One pose (multi-tag), use directly
            cameraPose =
                new Pose3d(
                    values[2],
                    values[3],
                    values[4],
                    new Rotation3d(new Quaternion(values[5], values[6], values[7], values[8])));
            robotPose3d =
                cameraPose.transformBy(GeomUtil.toTransform3d(cameraPoses[instanceIndex]).inverse());
                
            useVisionRotation = true;
            break;
          case 2:
            // Two poses (one tag), disambiguate
            double error0 = values[1];
            double error1 = values[9];
            Pose3d cameraPose0 =
                new Pose3d(
                    values[2],
                    values[3],
                    values[4],
                    new Rotation3d(new Quaternion(values[5], values[6], values[7], values[8])));
            Pose3d cameraPose1 =
                new Pose3d(
                    values[10],
                    values[11],
                    values[12],
                    new Rotation3d(new Quaternion(values[13], values[14], values[15], values[16])));
            Pose3d robotPose3d0 =
                cameraPose0.transformBy(GeomUtil.toTransform3d(cameraPoses[instanceIndex]).inverse());
            Pose3d robotPose3d1 =
                cameraPose1.transformBy(GeomUtil.toTransform3d(cameraPoses[instanceIndex]).inverse());

            // Check for ambiguity and select based on estimated rotation
            if (error0 < error1 * ambiguityThreshold || error1 < error0 * ambiguityThreshold) {
              Rotation2d currentRotation =
                  RobotState.getInstance().getEstimatedPose().getRotation();
              Rotation2d visionRotation0 = robotPose3d0.toPose2d().getRotation();
              Rotation2d visionRotation1 = robotPose3d1.toPose2d().getRotation();
              if (Math.abs(currentRotation.minus(visionRotation0).getRadians())
                  < Math.abs(currentRotation.minus(visionRotation1).getRadians())) {
                cameraPose = cameraPose0;
                robotPose3d = robotPose3d0;
              } else {
                cameraPose = cameraPose1;
                robotPose3d = robotPose3d1;
              }
            }
            break;
        }

        // Exit if no data
        if (cameraPose == null || robotPose3d == null) {
          continue;
        }

        // Exit if robot pose is off the field
        if (robotPose3d.getX() < -fieldBorderMargin
            || robotPose3d.getX() > FieldConstants.fieldLength + fieldBorderMargin
            || robotPose3d.getY() < -fieldBorderMargin
            || robotPose3d.getY() > FieldConstants.fieldWidth + fieldBorderMargin
            || robotPose3d.getZ() < -zMargin
            || robotPose3d.getZ() > zMargin) {
          continue;
        }
        
        // Get 2D robot pose
        Pose2d robotPose = robotPose3d.toPose2d();

        // Get tag poses and update last detection times
        List<Pose3d> tagPoses = new ArrayList<>();
        for (int i = (values[0] == 1 ? 9 : 17); i < values.length; i++) {
          int tagId = (int) values[i];
          lastTagDetectionTimes.put(tagId, Timer.getFPGATimestamp());
          Optional<Pose3d> tagPose = FieldConstants.getAprilTags().getTagPose((int) values[i]);
          tagPose.ifPresent(tagPoses::add);
        }

        // Calculate average distance to tag
        double totalDistance = 0.0;
        for (Pose3d tagPose : tagPoses) {
          totalDistance += tagPose.getTranslation().getDistance(cameraPose.getTranslation());
        }
        double avgDistance = totalDistance / tagPoses.size();
        // System.out.println(System.currentTimeMillis());
        // Add observation to list
        double xyStdDev = 1;
        if(edu.wpi.first.wpilibj.RobotState.isAutonomous()){
          xyStdDev = 3.3 * xyStdDevCoefficient.get() * Math.pow(avgDistance, 2.0) / tagPoses.size();
        }else{
          xyStdDev = xyStdDevCoefficient.get() * Math.pow(avgDistance, 2.0) / tagPoses.size();
        }
        double thetaStdDev =
            useVisionRotation
                ? thetaStdDevCoefficient.get() * Math.pow(avgDistance, 2.0) / tagPoses.size()
                : Double.POSITIVE_INFINITY;
        Logger.recordOutput("AprilTagVision/Inst" + instanceIndex + "/STDX", xyStdDev);
        Logger.recordOutput("AprilTagVision/Inst" + instanceIndex + "/STDT", thetaStdDev);
        allVisionObservations.add(
            new VisionObservation(
                robotPose, timestamp, VecBuilder.fill(xyStdDev, xyStdDev, thetaStdDev)));
        allRobotPoses.add(robotPose);
        allRobotPoses3d.add(robotPose3d);

        // Log data from instance
        Logger.recordOutput(
            "AprilTagVision/Inst" + instanceIndex + "/LatencySecs",
            Timer.getFPGATimestamp() - timestamp);
        Logger.recordOutput("AprilTagVision/Inst" + instanceIndex + "/RobotPose", robotPose);
        
        Logger.recordOutput("AprilTagVision/Inst" + instanceIndex + "/RobotPose3d", robotPose3d);
        Logger.recordOutput(
            "AprilTagVision/Inst" + instanceIndex + "/TagPoses", tagPoses.toArray(Pose3d[]::new));
      }

      // If no frames from instances, clear robot pose
      if (inputs[instanceIndex].timestamps.length == 0) {
        // Logger.recordOutput("AprilTagVision/Inst" + instanceIndex + "/RobotPose", new Pose2d());
        // Logger.recordOutput("AprilTagVision/Inst" + instanceIndex + "/RobotPose3d", new Pose3d());
      }

      // If no recent frames from instance, clear tag poses
      if (Timer.getFPGATimestamp() - lastFrameTimes.get(instanceIndex) > targetLogTimeSecs) {
        //noinspection RedundantArrayCreation
        // Logger.recordOutput("AprilTagVision/Inst" + instanceIndex + "/TagPoses", new Pose3d[] {});
      }
    }

    // Log robot poses
    Logger.recordOutput("AprilTagVision/RobotPoses", allRobotPoses.toArray(Pose2d[]::new));
    Logger.recordOutput("AprilTagVision/RobotPoses3d", allRobotPoses3d.toArray(Pose3d[]::new));

    // Log tag poses
    List<Pose3d> allTagPoses = new ArrayList<>();
    for (Map.Entry<Integer, Double> detectionEntry : lastTagDetectionTimes.entrySet()) {
      if (Timer.getFPGATimestamp() - detectionEntry.getValue() < targetLogTimeSecs) {
        allTagPoses.add(FieldConstants.getAprilTags().getTagPose(detectionEntry.getKey()).get());
      }
    }




    // Send results to robot state
    
    Logger.recordOutput("Number of updates",allVisionObservations.size());
    // if(allVisionObservations.size()>1){
    //   allVisionObservations = allVisionObservations.subList(0, 1);
    // }

    int maxObservations = 10;
    if(allVisionObservations.size()>maxObservations){
      allVisionObservations = allVisionObservations.subList(0, 10);
    }
    
    allVisionObservations.stream()
        .sorted(Comparator.comparingDouble(VisionObservation::timestamp))
        .forEach(RobotState.getInstance()::addVisionObservation);
         // Logger.recordOutput("AprilTagVision/TagPoses", allTagPoses.toArray(Pose3d[]::new));
    double totalMovement = Math.abs(RobotState.getInstance().getDrive().getReplayChassisSpeeds().omegaRadiansPerSecond) + Math.abs(RobotState.getInstance().getDrive().getChassisSpeeds().vxMetersPerSecond) + Math.abs(RobotState.getInstance().getDrive().getChassisSpeeds().vyMetersPerSecond);
    Logger.recordOutput("AprilTagVision/isStopped", totalMovement < 0.2);
    Logger.recordOutput("AprilTagVision/totaleMovement", totalMovement);
    if(totalMovement < 0.2){
      for(VisionObservation visionObservation: allVisionObservations) {
        averageStoppedPositions.add(visionObservation.visionPose());
      }
      double averageX = 0;
      double averageY = 0;
      double averageTheta = 0;
      for(Pose2d pose: averageStoppedPositions){
        averageX += pose.getX();
        averageY += pose.getY();
        averageTheta += pose.getRotation().getRadians();
      }
      averageX = averageX/averageStoppedPositions.size();
      averageY = averageY/averageStoppedPositions.size();
      averageTheta = averageTheta/averageStoppedPositions.size();
      Pose2d averagePosition = new Pose2d(averageX, averageY, new Rotation2d(averageTheta));
      Logger.recordOutput("AprilTagVision/averageStoppedPosition", averagePosition);
      RobotState.getInstance().setStoppedPosition(averagePosition);

  }else {
    averageStoppedPositions.clear();
    RobotState.getInstance().setStoppedPosition(null);
  }
  }
}
